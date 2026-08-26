# Changelog

All notable changes to this project will be documented in this file. See [conventional commits](https://www.conventionalcommits.org/) for commit guidelines.

---

## Unreleased

### Added
- Add AIA IMSIC IPI backend support for RustSBI Prototyper.
- Add SpacemiT K1 SoC platform support for RustSBI Prototyper, including OrangePi RV2 board configuration.
- Add SpacemiT K3 SoC platform support for RustSBI Prototyper, including K3 board configurations (K3 Pico-ITX / CoM260 / CoM260 IFX / DeepComputing FML13V05). Mirrors the vendor OpenSBI `spacemit_k3.c` capability set: A100 cluster parking (`boot_entry_dummy` equivalent + core8 wakeup), C2/C3 RVBADDR + hardware L2 flush, CCI-550 snoop, power-domain voting (vote/devote core+cluster, APCR VETE), DMASYS enable, RCPU runtime memory-region protection, REGISTER_PRESERVATION S-mode load/store emulation, and IMSIC suspend/resume state save/restore. Register definitions and init flow reference the official `spacemit-com/opensbi` `k3-br-v1.0.y` branch (commit `4869910`).
- Wire the platform hooks that the K3 feature set needs: trap-layer Load/Store access-fault emulation for REGISTER_PRESERVATION (`access_fault_handler` + `spacemit_k3::emulate_load/store`, mirroring OpenSBI's `emulate_load/store`), the system-suspend sequence (`suspend_pre` + `suspend` in `SbiSuspend::system_suspend`, mirroring `__rpmi_hsm_suspend_pre`/`__rpmi_hsm_suspend`), and the K3 PMP layout in `firmware::set_pmp` denying S-mode access to the RCPU runtime/DTB windows (mirroring `sbi_domain_root_add_memrange(..., ENF_PERMISSIONS)`).
- Add the SBI v2.0 extension set to the dispatcher: `fwft` (`SbiFwft`), `dbtr` (`SbiDbtr`), `cppc` (`SbiCppc`), `sse` (`SbiSse`) and `mpxy` (`SbiMpxy`), backed by the new `rpmi` crate (RPMI message protocol + shared-memory queue transport + mailbox abstraction, mirroring OpenSBI `lib/utils/mailbox`). The `mpxy` mailbox backend is injected per-platform via `SbiMpxy::set_mailbox`.
- Complete the RPMI mechanism layer in the `rpmi` crate: Base service-group client (`BaseClient`: probe service group, get spec/implementation version, get platform info), Clock/Voltage/Domain/CPPC service-group message structures with layout tests, and a shareable mailbox controller (atomic token allocation, `&self` operations so MPXY and CPPC share one instance).
- Complete the MPXY extension (`SbiMpxy`): shared-memory setup with OVERWRITE mode and disable, channel-ID enumeration, channel attribute read/write, and RPMI message send with/without response (MPXY `channel_id` = RPMI service group ID, MPXY `message_id` = RPMI `service_id`, mirroring OpenSBI `fdt_mpxy_rpmi_mbox.c`).
- Implement the CPPC extension over RPMI (`SbiCppc`): `probe`/`read`/`read_hi`/`write` forwarded to the RPMI CPPC service group (`PROBE_REG`/`READ_REG`/`WRITE_REG`).
- Discover the `riscv,rpmi-shmem-mbox` device-tree node and inject the shared-memory mailbox into both MPXY and CPPC (`inject_rpmi_mailbox`, mirroring OpenSBI `fdt_mailbox_rpmi_shmem.c`: `riscv,slot-size` property, four queue regions with head/tail/buffer slots, `db-reg` doorbell).
- Implement the FWFT `MISALIGNED_EXC_DELEG` feature for real (`SbiFwft`): gated on `misa.S`, it toggles the misaligned load/store bits of `medeleg` (mirroring OpenSBI `sbi_fwft.c`); the remaining features (landing pad / shadow stack / double trap / PTE A/D / pointer masking) stay not supported as they depend on Zicfilp / Zicfiss / SVADU / Smnpm hardware.
- Implement the RPMI notification path: token-less notification receive on the P2A_REQ queue (`SmqQueue::receive_by_message_id` with service-ID wildcard, `RpmiMailbox::receive_notification`), `BaseClient::enable_notification`, and `SbiMpxy::get_notification_events` writing the events state block (REMAINING/RETURNED/LOST/RESERVED) plus the event payload at offset `0x10` (mirroring OpenSBI `__smq_rx` `no_rx_token` and the MPXY `GET_NOTIFICATION_EVENTS` shared-memory layout).
- Document the DBTR/SSE capability boundary: `SbiDbtr` reports `num_triggers = 0` (no Sdtrig hardware, as OpenSBI probes `tselect`/`tdata1`), and `SbiSse` stays not supported (no platform event table / state machine), matching OpenSBI behavior on harts without the respective hardware.

### Modified
- refactor(prototyper): unify build commands (#227)
- deps: update `sbi-spec` to version 0.0.10.
- test-kernel: update PMU flag parameter trait names.
- Refine CSR group comments.
- fix(prototyper): temporary PMU fix for possible S-mode DTB modification
- fix(prototyper): validate DBCN console shared memory range

### Removed
