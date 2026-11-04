# First power-on: Systemcore and the radio

From a Systemcore and a VH-109 radio out of the box to the robot program deployed, with each step's
check. Then the hardware's own pages take over: a [CANivore](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/canivore.md),
the [coprocessors and Visor](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/visor.md),
and [calibration](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/calibration.md).

The Systemcore steps are from WPILib's [SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting)
pages for the alpha software the template is on; the radio's from WPILib's
[radio guide](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-3/radio-programming.html)
and [Vivid-Hosting's](https://frc-radio.vivid-hosting.net/). Neither is confirmed on a production
Systemcore yet: where a step differs, those pages win, and this one should be fixed.

`TE.AM` is your team number split as WPILib does: team 1234 is `10.12.34.x`, team 254 is
`10.2.54.x`.

## 1. Systemcore: its image and team number

1. **Reach its page.** Power it, connect a laptop by USB-C, Ethernet or its Wi-Fi, and open
   `http://robot.local`.
2. **Its image.** Use the Systemcore image WPILib releases alongside the WPILib version in this
   template (`.wpilib/wpilib_preferences.json`'s `projectYear`): SystemcoreTesting's compatibility
   table says which. Update it from `http://robot.local/configure` with the `.llupdate` file for your
   hardware revision (from Limelight's `systemcore-os-public` releases), keeping everything when
   asked. A unit too old for that is flashed instead, as the same page describes.
3. **Its team number.** On its configuration page, *Change Team Number*. **Check:** the page shows
   it.

Every address below comes from that number, and so does the robot program's: Visor's
configuration names the robot by it. With none set, the robot leaves Visor alone and raises the
alert `Visor/noTeam`.

## 2. The radio

1. **Firmware and mode.** Connect a laptop to the radio's **DS** port, give it 12 V, and open
   `http://radio.local`. Update its firmware if WPILib's radio guide says yours is too old, then set
   *Robot Radio Mode*, your team number, and the WPA keys. At an event the radio kiosk does this
   instead; don't configure it by hand there.
2. **Its ports on the robot:**

   | Port | What plugs in |
   |---|---|
   | RIO | Systemcore's Ethernet |
   | AUX1, AUX2 | Coprocessors (or a switch for more of them), at 10/100 Mb/s |
   | DS | A laptop on a tether, in the pits or at home |

   AUX1 and AUX2 can give passive PoE, off by default (DIP switches 1 and 2 on the radio's
   underside). Leave them off unless a device is made for passive PoE: an Orange Pi isn't.
3. **Power** it from the PDH, as the game manual's wiring rules say.

## 3. The robot's addresses

| Address | What | How |
|---|---|---|
| `10.TE.AM.1` | The robot radio | Its own |
| `10.TE.AM.2` | Systemcore | From the radio (DHCP), or static with gateway `10.TE.AM.4` |
| `10.TE.AM.4` | The gateway: the field's access point, or an access point radio at home | |
| `10.TE.AM.5` | A driver station set static | Mask `255.0.0.0` |
| `10.TE.AM.6` to `.19` | Coprocessors and other static devices | Mask `255.255.255.0`, gateway `10.TE.AM.4` |
| `10.TE.AM.200` to `.219` | What the radio hands out | DHCP |

The Paddock starter's coprocessors are `10.TE.AM.11`, `.12`, ... with gateway `10.TE.AM.4`, and
each one's Spotter agent names the robot at `10.TE.AM.2`.

**Check, from a laptop on the radio** (its DS port, or its Wi-Fi): `ping 10.TE.AM.2` answers, and
`ping 10.TE.AM.11` answers once that coprocessor is flashed and plugged into AUX1 or AUX2.

## 4. The Driver Station and a deploy

1. **The Driver Station** WPILib releases with this template's version, set to the same team
   number.
2. **The project's team number,** once: `./gradlew setTeamNumber -PteamNumber=NNNN`.
3. **Deploy:** `./gradlew deploy`. **Check:** the Driver Station shows robot code, and the robot
   starts disabled.
4. **Look at its alerts** (a dashboard that shows WPILib's alerts, such as AdvantageScope or
   Elastic) before enabling: each one
   names what's wrong and where to look. On a first robot expect the CANivore's, if its package
   isn't installed yet, and the coprocessors' until they're flashed and plugged in.
