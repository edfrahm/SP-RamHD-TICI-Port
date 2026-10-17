![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

## 🌞 What is sunnypilot?
[sunnypilot](https://github.com/sunnyhaibin/sunnypilot) is a fork of comma.ai's openpilot, an open source driver assistance system. sunnypilot offers the user a unique driving experience for over 300+ supported car makes and models with modified behaviors of driving assist engagements. sunnypilot complies with comma.ai's safety rules as accurately as possible.

## Ram port shared data upload: privacy

This fork (sunnypilot on the comma 3 for Ram Heavy Duty trucks, [edfrahm/SP-RamHD-TICI-Port](https://github.com/edfrahm/SP-RamHD-TICI-Port))
can share a small table of driving numbers with the project, to help make cruise control smoother on these trucks.
It is off unless you switch on **SunnyPilot/Ram Port data Upload** (Settings, sunnylink page). That one toggle also
controls sunnylink's own uploader. It runs only on Ram Heavy Duty trucks.

**What is sent**, about 10 times per second while driving:
- speed and acceleration, accelerator pedal position, engine speed and engine torque
- transmission: current and requested gear, whether the torque converter (the fluid coupling between engine and gearbox)
  is locked, whether a shift is under way, transmission input and output speed
- brakes and towing: driver on the brake pedal, brakes applied, exhaust brake on, tow/haul mode on, trailer connected,
  fuel level
- the estimated road grade, combined weight and air drag, and how sure each estimate is
- the factory cruise control: engaged or not, set speed, its engine torque and braking requests, the following-distance
  setting
- the camera's view of a car ahead: how sure it is that there is one, its distance, speed and acceleration

Each file also states the truck platform name, the engine and transmission module part numbers, the software version,
a random key made fresh for each drive (it links to nothing else), the part and row counts, and the date (no time of day).

**Never sent:** video, audio, any location (no latitude, longitude, altitude, or Global Positioning System (GPS)
speed or heading), the vehicle identification number (VIN), the route name, or the comma dongle ID.

**Trimming:** the first 2 minutes and the last 2 minutes of every drive are never written to the device's storage, so
the data does not show where a drive starts or ends.

**Registration:** once, the device registers with its own device key, its hardware serial number and its
cellular-modem ID (IMEI, International Mobile Equipment Identity). The service checks that they look genuine and keeps
only keyed hashes of them (one-way fingerprints made with a secret key), never the numbers themselves. It answers with
a random **RamPort ID**, shown on the Device settings page. Every upload is signed with the device key.

**When:** only while the toggle is on, and only on an unmetered network (for example Wi-Fi, never a metered cellular
data plan). While driving, files are sent only if the Device page's Onroad Uploads button is on; otherwise they wait
until the truck is parked. Until then, files wait on the device (at most 500 MB; the oldest are deleted first).
Switching the toggle off stops recording at once (the rest of a drive in progress is thrown away) and nothing more
is sent.

**Where it goes:** to the project's upload service, which stores the files in a private SharePoint site run by the
project maintainer, Eric. The service accepts uploads only: it offers no way to read, list, change or delete data.

**Deletion:** open an issue at [github.com/edfrahm/SP-RamHD-TICI-Port](https://github.com/edfrahm/SP-RamHD-TICI-Port/issues)
with the RamPort ID shown on your Device page, and your data will be deleted.
*(PLACEHOLDER: Eric to confirm this contact method or name another one.)*

## 💭 Join our Community Forum
Join the official sunnypilot community forum to stay up to date with all the latest features and be a part of shaping the future of sunnypilot!
* https://community.sunnypilot.ai/

## Documentation
https://docs.sunnypilot.ai/ is your one stop shop for everything from features to installation to FAQ about the sunnypilot

## 🚘 Running on a dedicated device in a car
First, check out this list of items you'll need to [get started](https://community.sunnypilot.ai/t/getting-started-using-sunnypilot-in-your-supported-car/251).

## Installation
Next, refer to the sunnypilot community forum for [installation instructions](https://community.sunnypilot.ai/t/read-before-installing-sunnypilot/254), as well as a complete list of [Recommended Branch Installations](https://community.sunnypilot.ai/t/recommended-branch-installations/235).

## 🎆 Pull Requests
We welcome both pull requests and issues on GitHub. Bug fixes are encouraged.

Pull requests should be against the most current `master` branch.

## 📊 User Data

By default, sunnypilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

## Licensing

sunnypilot is released under the [MIT License](LICENSE). This repository includes original work as well as significant portions of code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot), which is also released under the MIT license with additional disclaimers.

The original openpilot license notice, including comma.ai’s indemnification and alpha software disclaimer, is reproduced below as required:

> openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.
>
> Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.
>
> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

## 💰 Support sunnypilot
If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.


<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>

Your continuous love and support are greatly appreciated! Enjoy 🥰

<span>-</span> Jason, Founder of sunnypilot
