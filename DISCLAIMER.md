# Disclaimer

Read this before you use anything in this repository.

## No warranty

This repository is provided "as is", without warranty of any kind, express or implied, including but not
limited to the warranties of merchantability, fitness for a particular purpose, accuracy, and
non-infringement. See [LICENSE](LICENSE).

In no event shall the author be liable for any claim, damages, injury, death, property damage, or other
liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with
this material or its use.

## No claims and no guarantees

Nothing here is a claim, a promise, or a guarantee.

Everything in this repository — the documentation, the code, the code comments, the commit messages, and the
video — records what one person observed on one specific vehicle during private experimentation. It is a
personal notebook that has been made public in case any of it is useful to someone else.

Specifically:

- Nothing here is claimed to be correct, complete, current, or safe.
- Nothing here is claimed to work on your vehicle, or on any vehicle.
- No statement about a signal, an address, a scale factor, a limit, or a behavior is a guarantee that your
  vehicle behaves the same way. Vehicles differ by market, model year, trim, and firmware version, and they
  change with software updates.
- Numbers, measurements, and observations may be wrong, may have been superseded, and are not reproducible
  claims. Verify everything on your own vehicle before you rely on it.
- Comments inside the code describe what the author observed while developing it. They are development notes,
  not assertions about your vehicle or assurances of correctness.
- Passing a test proves nothing beyond that test. There is no claim of validation, certification,
  qualification, or fitness for road use.
- No support, maintenance, updates, or fixes are offered or implied.

## Use at your own risk

This material concerns the steering, braking, and acceleration of a moving motor vehicle. An error, a wrong
assumption, or a difference between your vehicle and the one described here can cause loss of vehicle
control, with consequences for you and for other road users.

If you use any of it, you do so entirely at your own risk and on your own responsibility. You alone are
responsible for:

- Deciding whether any of this is appropriate for your vehicle and your situation.
- Remaining in control of your vehicle at all times, and supervising any system you build.
- Testing in a safe, private, controlled environment before any use on a public road.
- Complying with all laws, regulations, type-approval rules, and road traffic rules that apply where you are.
  Modifying a vehicle's advanced driver assistance systems, or the vehicle itself, is restricted or
  prohibited in many jurisdictions.
- Any consequence for your vehicle's warranty, insurance cover, roadworthiness certification, and legal
  liability. Modifying a vehicle is very likely to affect all of these.

Do not use any of this on a public road unless you are qualified to judge the risk and are prepared to accept
it in full.

## Not affiliated, not endorsed

The author is an independent private individual with no connection to any of the organizations named here.

This project is not affiliated with, authorized by, endorsed by, sponsored by, or connected to BYD Auto Co.,
Ltd., comma.ai, Veoneer, Magna, Robert Bosch GmbH, or any vehicle manufacturer, supplier, or their
subsidiaries.

"BYD", "Atto 3", and "Yuan Plus" are trademarks of BYD Auto Co., Ltd. "openpilot", "comma", "comma 3X",
"comma 4", and "panda" are trademarks of comma.ai, Inc. All trademarks are the property of their respective
owners and are used here only to identify the hardware and software being described. Their use does not imply
any association or endorsement.

## Third-party material and licensing

The code in [`port/`](port/) is derived work. It builds on, and in places is adapted from:

- [openpilot](https://github.com/commaai/openpilot) and [opendbc](https://github.com/commaai/opendbc) by
  comma.ai, MIT licensed.
- [carrotpilot](https://github.com/ajouatom/carrotpilot) by ajouatom, an openpilot fork. The files in
  `port/` target this fork's APIs.
- bukapilot by iXcess / KommuAI, an openpilot fork, which is the origin of the earlier BYD work this port
  started from.

Copyright in those portions remains with their respective authors under their respective licenses. The MIT
license in [LICENSE](LICENSE) covers this repository's own contribution and does not override or extend to
any third-party rights. If you believe material here is used incorrectly or should be attributed
differently, please open an issue and it will be corrected or removed.

Interface facts recorded here — pin positions, message addresses, signal layouts — were determined by
measurement and observation on a privately owned vehicle. They are stated as observations of fact. No
manufacturer document, source code, or other protected material is reproduced in this repository.
