![compilation workflow](https://github.com/frno7/cf68901/actions/workflows/compilation.yml/badge.svg)

# CF68901 multifunction peripheral (MFP)

The CF68901 module in C is designed to be compatible with the [MC68901]
multifunction peripheral (MFP) timer and interrupt controller made in 1979.
This device was often used with the [M68000 processor] in computers such as
the [Atari ST] in the 1980s.

The CF68901 repository is made to be included as a Git submodule in larger
designs, for example [PSG play](https://github.com/frno7/psgplay).

# Test and verification

`make verify` compiles tests and verifies. Each test is a
port command sequence (PCS). A trivial example testing a 200 Hz timer A:

```
CLK=4000000             -- 4.000000 MHz CLK
XTAL1=2457600           -- 2.457600 MHz XTAL1

RESET_L=0 #4
RESET_L=1 #4

#2 VR=0x40		-- Vector base 0x40 with SEI=0
#1 IMRA=0b00100000	-- Timer A mask enable
#1 IERA=0b00100000	-- Timer A enable
#1 TADR=192		-- Data counter 192 with prescale 64 is 200 Hz
#1 TACR=5		-- Control mode 5 is prescale 64, starting the timer

-- Assert that a timer A interrupt is requested after
-- exactly 64*192=12288 XTAL1 cycles = 20000 CLK cycles
#19999 IRQ_L!1		-- An interrupt is not yet asserted
    #1 VECTOR!0x4d	-- Acknowledge timer A interrupt on channel 13 (0xd)

#19999 IRQ_L!1
    #1 VECTOR!0x4d

#19999 IRQ_L!1
    #1 VECTOR!0x4d

-- ... The timer repeats indefinitely in 20000 CLK cycles ...
```

Commands are port assignments (`=`), clock cycle increments (`#`), and
assertions (`!`). Newlines and whitespace don’t matter, and comments
begins with `--`.

`make TRACE=1 verify` displays commands and events during verification.

The CF68901 implementation in C can be co-simulated, in cycle-for-cycle
lockstep, with other implementations in [VHDL] and [Verilog].

# Manuals and references

- The [Motorola MC68901 manual].
- The [Motorola MC68HC901 manual].
- The [NXP MC68HC901 manual].
- The [Hatari] emulator is well-researched with extensive source code comments.
- [MiSTery] has a Verilog implementation.
- [Zest] has a VHDL implementation.

[MC68901]: https://www.nxp.com/products/no-longer-manufactured/mc68hc901-multi-function-peripheral:MC68901
[M68000 processor]: https://en.wikipedia.org/wiki/Motorola_68000
[Atari ST]: https://en.wikipedia.org/wiki/Atari_ST

[Motorola MC68901 manual]: https://archive.org/details/Motorola_MC68901_MFP_undated
[Motorola MC68HC901 manual]: https://sca.uwaterloo.ca/coldfire/specs/HC901UM.pdf
[NXP MC68HC901 manual]: https://www.nxp.com/docs/en/reference-manual/MC68901UM.pdf

[Hatari]: https://github.com/hatari/hatari
[MiSTery]: https://github.com/gyurco/MiSTery
[Zest]: https://codeberg.org/zerkman/zest

[VHDL]: https://en.wikipedia.org/wiki/VHDL
[Verilog]: https://en.wikipedia.org/wiki/Verilog
