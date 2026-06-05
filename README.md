# WeActStudio.USB2CANFDV1

This repository contains documentation and an implementation of a protocol that extends the LAWICEL CAN ASCII protocol with additional commands for CAN FD and new features.
The implementation is for the [USB2CANFDV1](https://github.com/WeActStudio/WeActStudio.USB2CANFDV1) from WeAct Studio.  

URL to this repository: https://github.com/Nakakiyo092/usb2canfdv1


## Frequently used commands

- `O[CR]` - Opens the CAN channel
- `C[CR]` - Closes the CAN channel
- `sddxxyyzz[CR]` - Sets custom nominal bit rate
- `yddxxyyzz[CR]` - Sets custom CANFD data segment bit rate
- `tiiildd...[CR] `- Transmits base frame
- `Tiiiiiiiildd...[CR] `- Transmits extended frame
- `diiildd...[CR] `- Transmits CANFD base frame (BRS disabled)
- `Diiiiiiiildd...[CR] `- Transmits CANFD extended frames (BRS disabled)
- `biiildd...[CR] `- Transmits CANFD base frames (BRS enabled)
- `Biiiiiiiildd...[CR] `- Transmits CANFD extended frames (BRS enable)
- `V[CR]` and `v[CR]` - Returns firmware version and remote path as a string
- `Z[CR]` and `z[CR]` - Configures reporting mechanism including time stamp and Tx event
- `M[CR]` and `m[CR]` - Configures CAN acceptance filter
- `F[CR]` - Returns status flags

`[CR]` : `0x0D` (hex), `\r` (ascii)

Please find more information in the `doc` directory or the [wiki](https://github.com/Nakakiyo092/usb2canfdv1/wiki).


## Toolchain
The toolchain in this repository is designed to run on a Windows PC.

### How to build firmware
Use STM32CubeIDE 2.1.1 with FW_G0 V1.6.3.

### How to flash firmware
Use the packager tool and the upgrade tool in the [root repository](https://github.com/WeActStudio/WeActStudio.USB2CANFDV1/tree/master/Tools).

> [!NOTE]
> After flashing this firmware, there would be an issue upgrading to another firmware.
> You should force upgrade mode by shorting DIO and GND as described in the [root repository](https://github.com/WeActStudio/WeActStudio.USB2CANFDV1?tab=readme-ov-file#how-to-force-firmware-upgrade-mode).
> There would be no command to enter firmware upgrade mode.


## Credit

### Related work

| Name | Author | License |
|------|--------|---------|
| [canable2-fw](https://github.com/normaldotcom/canable2-fw) | Openlight Labs | GPL-3.0 |
| [WeActStudio.USB2CANFDV1](https://github.com/WeActStudio/WeActStudio.USB2CANFDV1) | WeAct Studio | See repository |

### Bundled third-party components

See LICENSE.md for the full list of bundled components and their licenses.
