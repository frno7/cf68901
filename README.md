![compilation workflow](https://github.com/frno7/cf68901/actions/workflows/compilation.yml/badge.svg)

# CF68901 multifunction peripheral (MFP)

The CF68901 module in C is designed to be compatible with the [MC68901]
multifunction peripheral (MFP) timer and interrupt controller made in 1979.
This device was often used with the [M68000 processor] in computers such as
the [Atari ST] in the 1980s.

The CF68901 repository is made to be included as a Git submodule in larger
designs, for example [PSG play](https://github.com/frno7/psgplay).

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
[Zest]: https://github.com/zerkman/zest
