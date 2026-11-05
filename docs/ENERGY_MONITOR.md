# Independent measured-energy monitoring

The user provided a 22.2V, 5700mAh battery with label energy 129.96Wh. These are recorded in `config/vio_battery.json`: voltage × capacity gives 126.54Wh, a 3.42Wh discrepancy from the supplied label energy. Both are retained, `nameplate_consistent` is false, and the lower 126.54Wh is reported as `nominal_reference_wh`. This reference is not automatically used as starting usable energy.

The monitor reports **measured consumed Wh and mAh**. Actual usable capacity and starting charge are still unknown. `remaining_usable_wh` and `energy_sufficient` remain JSON null until usable starting energy, next-task budget and reserve are explicitly provided. Task code must require `energy_sufficient is True`; null is not permission to launch. Status includes `battery_profile` metadata; `--battery-profile` can override the default file.

```
consumed_Wh += 0.5 * (previous_voltage * previous_current
                    + current_voltage * current_current) * dt_seconds / 3600
discharged_mAh += 0.5 * (previous_current + current_current) * dt_seconds / 3.6
```

These are trapezoidal integrals of sampled battery measurements. Their accuracy depends on voltage/current calibration and sample coverage. A few flights cannot establish full pack capacity, starting charge, aged usable capacity or load-dependent voltage collapse. The instantaneous voltage/current, cell health and PX4 protection remain relevant; accumulated energy alone is not an adequate electrical failsafe.

The model intentionally ignores PX4's `remaining` field. It integrates through flight, landing and another takeoff without resetting on arm-state changes. Battery replacement/new charge requires a new explicit pack/session ID and journal; it is not inferred from a momentary voltage rebound.

Invalid/disconnected current data, backwards clocks, >3s source/receipt gaps or a source/monotonic timing mismatch break coverage. Measured energy remains available as a partial total, but a complete remaining-energy budget is no longer certified. Repeated source timestamps do not refresh the deadline. Restart restores the journal; a host reboot invalidates complete coverage because unobserved energy cannot be reconstructed. The journal is not permission to count a physically different battery as the same one.

## Optional runtime

No new service is enabled by this change. After sourcing ROS2 and px4_msgs, a manually started read-only process can use the observed historical source ID 1:

```bash
python3 tools/vio_energy_monitor.py \
  --source-id 1 --battery-id PACK_AND_CHARGE_SESSION_ID \
  --state-file /home/nano/vio_energy/PACK_AND_CHARGE_SESSION_ID.json
```

It subscribes only to `/fmu/out/battery_status` (with the msg version suffix if defined) and publishes `/vio/energy_status` at 2Hz as std_msgs/String JSON. DDS domain is inherited from ROS_DOMAIN_ID; `--topic-prefix` applies to both topics. It emits no vehicle commands. Use the actual pack/session name and validate the source ID on the current firmware before deployment.

Later, provide **usable remaining Wh at the beginning of this accounting session**, `--initial-usable-wh`, plus `--next-task-wh` and `--reserve-wh`. Nominal mAh times nominal voltage is a nameplate estimate, not automatically a validated usable budget. Supplying initial usable Wh after an earlier unmeasured discharge cannot repair missing coverage; reconcile the session explicitly.

## Recorded-data check

Using the raw logged V and I:

| ULog | Covered duration | Measured consumption | Integrated charge |
|---|---:|---:|---:|
| 13_47_11.ulg | 287.20s | 40.966Wh | 1999.47mAh |
| 14_07_14.ulg | 44.80s | 5.809Wh | 284.87mAh |

This is consumption during each recorded interval, not capacity or charge at takeoff. PX4's discharged_mAh endpoint differences are 1999.44mAh and 278.55mAh respectively; the second differs by about 2.3%, so the two methods must not be described as identical. Endpoint semantics, filtering/integration and calibration should be checked before using a tight energy reserve.

`vio_energy_regression` tests known integrals, unknown capacity, multi-leg continuity, invalid readings, gaps, timestamp replay, journal restart/pack mismatch and reserves. `vio_energy_ros_regression` uses DDS domain45, a mock battery source, capacity unknown and a simulated stream gap. It does not test the electrical hardware or prove the current sensor's accuracy.
