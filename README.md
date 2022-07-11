# Energesis_SHT2x

Arduino library for the SHT2x, HTU2x and Si70xx series temperature and humidity sensor.

## Compatible sensors

List of compatible sensors:

- SHT20
- SHT21
- SHT22
- HTU20D(F)
- HTU21D(F)
- Si7013
- Si7020
- Si7021

## Classes

The following table shows which class to use for each sensor.

| Class | Header | Sensors | Example |
| -- | -- | -- | -- |
| Energesis_SHT2x | `Energesis_SHT2x.h` | SHT20, SHT21, SHT22  | [SHT2x Full](https://github.com/Energesis-Ingenieria/Energesis_SHT2x/blob/main/examples/SHT2x_full/SHT2x_full.ino) |
| Energesis_HTU2xD | `Energesis_HTU2xD.h` |  HTU20D(F), HTU21D(F) | [HTU2x Full](https://github.com/Energesis-Ingenieria/Energesis_SHT2x/blob/main/examples/HTU2x_full/HTU2x_full.ino), [HTU2x Generic](https://github.com/Energesis-Ingenieria/Energesis_SHT2x/blob/main/examples/HTU2x_generic/HTU2x_generic.ino) |
| Energesis_Si7021 | `Energesis_Si7021.h` | Si7013, Si7020, Si7022 |  |
