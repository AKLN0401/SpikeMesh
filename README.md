# SpikeMesh

A small neuromorphic processor: a **2x2 mesh of spiking-neuron cores** connected by
**Address-Event Representation (AER)** routers. Each core holds 256 leaky integrate-and-fire (LIF)
neurons, a programmable synapse memory, and a routing table that sends spikes to any core in the mesh.

- Plain **Verilog-2001**, one clock `clk`, asynchronous active-low reset `rst_n`
- 4 cores x 256 neurons = **1024 neurons**, 4 x 1024 synapses
- XY-routed 5-port routers with valid/ready back-pressure (no packet is ever dropped)
- Simulated in Vivado (XSim) and Cadence Xcelium; taken through Cadence Genus/Innovus on TSMC 65 nm

Engineer: Khoi Nguyen

---

## Top-level block diagram (`aer_mesh2x2`)

```
                 cfg_we/bank/addr/data (per core)      tick (global)
                 ───────────────┬──────────────────────────┬──────────
                                │                          │
      y=1   ┌───────────────────┴──┐              ┌────────┴─────────────┐
            │  neurocore 2 (0,1)   │              │  neurocore 3 (1,1)   │
            └──────────┬───────────┘              └──────────┬───────────┘
                       │ Local                               │ Local
                 ┌─────┴─────┐       East / West       ┌─────┴─────┐
                 │ router r2 │◄───────────────────────►│ router r3 │
                 └─────┬─────┘                         └─────┬─────┘
                       ▲ N/S                                 ▲ N/S
                       ▼                                     ▼
                 ┌─────┴─────┐       East / West       ┌─────┴─────┐
                 │ router r0 │◄───────────────────────►│ router r1 │
                 └─────┬─────┘                         └─────┬─────┘
                       │ Local                               │ Local
      y=0   ┌──────────┴───────────┐              ┌──────────┴───────────┐
            │  neurocore 0 (0,0)   │              │  neurocore 1 (1,0)   │
            └──────────────────────┘              └──────────────────────┘
                      x=0                                   x=1

   core index i = 2*y + x          outputs: core_idle[3:0], tick_missed[3:0], net_active
```

Router ports: `0 = Local, 1 = North, 2 = East, 3 = South, 4 = West`. Edge ports with no
neighbour are tied off. x grows East, y grows North.

### Top-level ports

| Port | Dir | Width | Description |
|---|---|---|---|
| `clk` | in | 1 | Single clock for the whole design |
| `rst_n` | in | 1 | Asynchronous active-low reset (control logic only; memories are not reset) |
| `cfg_we` | in | 4 | Config write enable, one bit per core |
| `cfg_bank` | in | 8 | 2 bits per core: which memory to write (see below) |
| `cfg_addr` | in | 40 | 10 bits per core: entry address |
| `cfg_data` | in | 156 | 39 bits per core: entry data |
| `tick` | in | 1 | Global timestep pulse: every core sweeps all its neurons |
| `core_idle` | out | 4 | Core i holds no spike, event or packet |
| `tick_missed` | out | 4 | Sticky: a tick arrived while core i was still busy |
| `net_active` | out | 1 | A router input or output is carrying a packet this cycle |

Parameters: `NEU_AW = 8` (256 neurons/core), `SPK_FIFO = 1` (spike FIFO in each core),
`RAW = 2` (router input FIFO depth 4).

---

## Neurocore (`neurocore`)

```
 pkt_in ──► axon_decode ──► syn_sram ──► neuron_unit ──► [spike FIFO] ──► spike_gen ──► pkt_out
 (router      axon LUT       synapse      LIF datapath     aer_fifo        spike table   (router
  Local out)  256 x 16       1024 x 16    + state 256x39   2^NEU_AW deep   256 x 15      Local in)
                ▲               ▲              ▲                               ▲
                └───────────────┴──── host config (cfg_bank 0..3) ─────────────┘
```

1. **axon_decode** - an incoming packet carries an axon id. The axon LUT gives
   `{base, count}`: the first synapse address and the fan-out (up to 63 synapses).
   It then emits `count` synapse read requests.
2. **syn_sram** - each synapse entry is `{target_neuron[7:0], weight[7:0]}` (signed weight).
3. **neuron_unit** - adds the weight to the target neuron's membrane potential (saturating).
   On `tick` it sweeps every neuron, applies leak/threshold/refractory and emits spikes.
4. **spike FIFO** - holds up to 256 spikes so a tick sweep never stalls on network back-pressure.
5. **spike_gen** - looks up the firing neuron's routing entry and builds the AER packet.
   A neuron whose entry has `en = 0` does not send a packet.

### Configuration banks (`cfg_bank`)

| Bank | Memory | Entry format | `cfg_addr` |
|---|---|---|---|
| 0 | Axon LUT | `{base[9:0], count[5:0]}` | axon id |
| 1 | Synapse SRAM | `{target[7:0], weight[7:0]}` | synapse address |
| 2 | Neuron state | `{refr[2:0], leak[3:0], thr[15:0], v[15:0]}` | neuron id |
| 3 | Spike table | `{en, dest_x, dest_y, axon_id[7:0], delay[3:0]}` | neuron id |

Write only while the core is idle. **The memories have no reset**: the host must write every
neuron (banks 2 and 3) and every axon/synapse that will be used before the first tick.

---

## Neuron model (LIF, `neuron_unit`)

Discrete time, one timestep = one `tick`:

```
event (any time):     V <- sat(V + weight)                 integrate, no threshold test

on tick, for each neuron n = 0..255:
    if refr != 0:     refr <- refr - 1;  V <- 0            refractory
    else:             V_l = V - (V >>> leak)               leak (arithmetic shift)
                      if V_l >= thr:  spike(n); V <- 0; refr <- REFR_P   (REFR_P = 2)
                      else:           V <- V_l
```

- `V`, `thr`: 16-bit signed. `leak`: 4-bit shift (`leak = 1` halves V each tick; `leak = 15`
  is effectively no leak; `leak = 0` clears V every tick).
- One event takes 2 cycles; a tick sweep takes 2 cycles per neuron (~512 cycles per core).
- A tick that arrives while a sweep is still pending is dropped and sets `tick_missed`.

---

## AER packet and router (`aer_router`)

16-bit packet:

```
 15  14   13       12       11 ........ 4   3 ..... 0
┌──────┬────────┬────────┬───────────────┬──────────┐
│ type │ dest_x │ dest_y │  axon_id[7:0] │ delay    │
└──────┴────────┴────────┴───────────────┴──────────┘
```

- **5 input FIFOs** (`aer_fifo`, first-word-fall-through, depth `2^AW`): the header is visible
  before the pop, so the route is computed directly from the FIFO head.
- **XY routing** (dimension order, deadlock-free on a mesh): first along x, then along y:
  `dest_x > X -> East`, `< X -> West`, else `dest_y > Y -> North`, `< Y -> South`, else `Local`.
- **Per-output round-robin arbiter** between the inputs that request that output.
- **Registered outputs** with valid/ready: a stalled neighbour holds the packet (back-pressure),
  so packets are never lost.

---

## Operating sequence

1. Reset (`rst_n = 0`, then 1).
2. Configure every core: neuron states (bank 2), spike tables (bank 3), axon LUT (bank 0),
   synapses (bank 1).
3. Wait until `&core_idle && !net_active`, then pulse `tick` for one cycle.
4. Spikes travel through the mesh and are integrated into the target neurons. Repeat from 3.

Example (2-hop path): neuron 0 of core 0 has spike table entry `{en=1, dest=(1,1), axon=5}`.
Its packet goes r0 -> East -> r1 -> North -> r3 -> Local -> core 3. Core 3's axon LUT entry 5
lists its synapses, and their weights are added to the target neurons there.

---

## Repository layout

```
rtl/
  aer_mesh2x2.v    top: 4 neurocores + 4 routers, mesh links, edge tie-offs
  neurocore.v      one core: decode -> synapse -> neuron -> FIFO -> spike_gen
  axon_decode.v    axon LUT, fan-out address generator
  syn_sram.v       synapse memory with valid/ready read stage
  neuron_unit.v    time-multiplexed LIF datapath + neuron state memory
  spike_gen.v      spike table, neuron spike -> AER packet
  aer_router.v     5-port XY router, input FIFOs, round-robin arbiters
  aer_fifo.v       FWFT FIFO (router inputs and core spike FIFO)
tb/
  tb_aer_router.v  router: XY routes, arbitration, back-pressure
  tb_axon_decode.v axon LUT and fan-out bursts
  tb_syn_ram.v     synapse memory read/write
  tb_syn_path.v    axon_decode + syn_sram
  tb_neuron_unit.v LIF unit against a behavioural integer model
  tb_snn_path.v    decode -> synapse -> neuron
  tb_spike_gen.v   spike table and packet build
  tb_mesh.v        full 2x2 mesh: config, ticks, multi-hop spikes
```

Module hierarchy:

```
aer_mesh2x2
├── neurocore  g_core[0..3]
│   ├── axon_decode
│   ├── syn_sram
│   ├── neuron_unit
│   ├── aer_fifo       (spike FIFO, SPK_FIFO = 1)
│   └── spike_gen
└── aer_router u_r0..u_r3
    └── aer_fifo x5    (one per input port)
```

## Simulation

Vivado: add `rtl/*.v` as design sources and `tb/*.v` as simulation sources, set the testbench
(e.g. `tb_mesh`) as top, and run behavioural simulation. Each testbench prints `PASS` / `FAIL`
lines and a summary.

Xcelium:

```
xrun rtl/*.v tb/tb_mesh.v -top tb_mesh -access +r
```

## ASIC implementation (in progress)

TSMC 65 nm CLN65GP+ (`tcbn65gplus`), Cadence Genus 25.1 (iSpatial) and Innovus 25.1:

- Core memories mapped to ARM dual-port SRAM macros (16 macros); router/spike FIFOs stay in flops.
- An SPI configuration slave reduces the chip to 7 signal pins
  (`clk, rst_n, sclk, cs_n, mosi, miso, all_idle`).
- Current trial at a 10 ns clock: setup and hold met, ~0.75 mm² cell area, ~28 mW.
