# BlueOS-Water-Linked-DVL

## Changelog

### v1.0.10
 - Improve DVL finding logic
 - Refactor settings file

### v1.0.7
  - Fix using lat/long inputs with no internet

### v1.0.6
 - No longer sets parameters automatically. Users can now change for two modes of operation:
     - DVL-only, the recommended mode
     - DVL+GPS, experimental mode which allows fusing (underwater) GPS and DVL data

### v1.0.5
 - Update texts to make support of DVL A125 obvious

### v1.0.4
 - Fix issue introduced in v1.0.3 where the extension was unable to talk to Cable-guy

### v1.0.3
 - Uses an random available port instead of 9001 to avoid conflict
 - Updated menu icon

### v1.0.2
 - Improved style

### v1.0.1
 - Fixed an issue where the driver was sending Rangefinder messages with invalid data

This is a docker implementation of a Water Linked DVL A50 and A125 driver as a BlueOS Extension.

## Install

Install it from [BlueOS extensions tab](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS-1.1/extensions/).

The service will show in the "Extension Manager" section in BlueOS, where there are some configuration options.

## Mounting rotation

The DVL does not have to point straight down. Set its mounting angles relative to the vehicle
frame on the extension page, or through the REST API:

 - `GET /mount_pitch/<degrees>`: DVL pitch in [-180, 180]. **Positive = DVL tilted nose-up relative to the frame**
   (a rotation about the vehicle's starboard axis that lifts the DVL's LED side up).
   0° is the old "Down" orientation, 90° is the old "Forward" orientation (transducers facing forward).
 - `GET /mount_roll/<degrees>`: DVL roll in [-180, 180]. Positive = DVL rolled right side down.
 - `GET /orientation/<1|2>`: presets, 1 = Down (pitch 0°), 2 = Forward (pitch 90°). Both set roll to 0°.

The angles are saved as `mount_pitch_deg` and `mount_roll_deg` in the settings file, and older settings files that only
have `orientation` are migrated automatically. The rotation (pitch first, then roll) is applied to the velocity sent
as `VISION_POSITION_DELTA` or `VISION_SPEED_ESTIMATE`, to the attitude deltas, and to the rangefinder distance, which is
projected onto the vehicle's down axis and not sent when the DVL is not looking down.
