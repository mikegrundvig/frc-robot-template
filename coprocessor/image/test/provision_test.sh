# shellcheck shell=bash
# provision.sh, offline, on a tree shaped like PhotonVision's Armbian image: what it changes, that a
# second run changes nothing, and what it refuses. (The package install and the smoke test run only
# in the image's chroot, in CI.)

setup_provision() {
  need systemctl
  make_image_root
}

test_provision_makes_the_root_read_only_and_data_writable() {
  setup_provision
  run_provision
  local fstab=$TMP/root/etc/fstab
  grep -Eq '^UUID=0b1c2d3e-0000-4000-8000-000000000000 / ext4 ro,noatime 0 1$' "$fstab" ||
    fail "the root's line isn't read-only: $(grep ' / ' "$fstab")"
  assert_contains "$fstab" 'LABEL=coproc-data /data ext4 noatime,errors=remount-ro,nofail,x-systemd.device-timeout=10s 0 2'
  assert_contains "$fstab" '/data/photonvision_config /opt/photonvision/photonvision_config none bind'
  assert_contains "$fstab" '/data/journal /var/log/journal none bind'
  local point
  for point in /tmp /var/tmp /var/log /var/lib/systemd /var/lib/NetworkManager; do
    grep -Eq "^tmpfs $point tmpfs " "$fstab" || fail "$point isn't on tmpfs"
  done
  # The base image's own /tmp line is replaced; lines for other mount points stay.
  assert_eq "$(grep -c ' /tmp ' "$fstab")" 1 "lines for /tmp"
  assert_contains "$fstab" 'proc /proc proc defaults 0 0'
  [[ -d $TMP/root/data && -d $TMP/root/opt/photonvision/photonvision_config ]] || fail "no mount points"
}

test_provision_keeps_the_journal_and_syncs_it_often() {
  setup_provision
  run_provision
  local conf=$TMP/root/etc/systemd/journald.conf.d/70-coprocessor.conf
  assert_contains "$conf" 'Storage=persistent'
  assert_contains "$conf" 'SystemMaxUse=256M'
  assert_contains "$conf" 'SyncIntervalSec=10s'
}

test_provision_turns_off_armbians_ram_logging_resize_and_first_run() {
  setup_provision
  run_provision
  assert_contains "$TMP/root/etc/default/armbian-ramlog" 'ENABLED=false'
  assert_file "$TMP/root/root/.no_rootfs_resize"
  local unit
  for unit in armbian-ramlog.service armbian-resize-filesystem.service armbian-firstrun.service \
    apt-daily.timer logrotate.timer; do
    assert_masked "$TMP/root" "$unit"
  done
  assert_contains "$TMP/root/etc/e2fsck.conf" 'broken_system_clock = 1'
  [[ ! -s $TMP/root/etc/machine-id ]] || fail "the common image carries a machine ID"
}

test_provision_turns_off_photonvisions_network_management() {
  setup_provision
  run_provision
  local dropin=$TMP/root/etc/systemd/system/photonvision.service.d/10-coprocessor.conf
  assert_contains "$dropin" 'ExecStart=/usr/bin/java -Xmx512m -jar /opt/photonvision/photonvision.jar -n'
  assert_contains "$dropin" 'RequiresMountsFor=/opt/photonvision/photonvision_config'
  assert_eq "$(grep -c '^ExecStart=$' "$dropin")" 1 "ExecStart resets"
  cmp -s "$TMP/root/opt/photonvision/photonvision.jar" "$TMP/photonvision.jar" || fail "the jar wasn't installed"
}

test_provision_leaves_the_network_to_the_stamped_profile() {
  setup_provision
  run_provision
  assert_contains "$TMP/root/etc/NetworkManager/conf.d/90-coprocessor.conf" 'no-auto-default=*'
  assert_no_file "$TMP/root/etc/netplan/10-dhcp-all-interfaces.yaml"
  assert_file "$TMP/root/etc/netplan.disabled/10-dhcp-all-interfaces.yaml"
  assert_no_file "$TMP/root/etc/NetworkManager/system-connections/old.nmconnection"
  assert_masked "$TMP/root" systemd-networkd.service
}

test_provision_makes_ssh_key_only_with_a_host_key_per_drive() {
  setup_provision
  run_provision
  local conf=$TMP/root/etc/ssh/sshd_config.d/10-coprocessor.conf
  assert_contains "$conf" 'PasswordAuthentication no'
  assert_contains "$conf" 'KbdInteractiveAuthentication no'
  assert_contains "$conf" 'HostKey /data/ssh/ssh_host_ed25519_key'
  assert_no_file "$TMP/root/etc/ssh/ssh_host_ed25519_key"
  assert_no_file "$TMP/root/etc/ssh/ssh_host_ed25519_key.pub"
  [[ -L $TMP/root/etc/systemd/system/ssh.service.wants/coprocessor-ssh-hostkey.service ]] ||
    fail "the host key service isn't enabled"
}

test_provision_installs_the_agent() {
  setup_provision
  run_provision
  cmp -s "$TMP/root/opt/coprocessor/coprocessor-agent.jar" "$TMP/coprocessor-agent.jar" ||
    fail "the agent's jar wasn't installed"
  cmp -s "$TMP/root/etc/systemd/system/coprocessor-agent.service" "$TMP/coprocessor-agent.service" ||
    fail "the agent's unit wasn't installed"
  local wants=$TMP/root/etc/systemd/system/multi-user.target.wants
  [[ -L $wants/coprocessor-agent.service ]] || fail "the agent isn't enabled"
  [[ -L $wants/coprocessor-facts.service ]] || fail "the facts helper isn't enabled"
  [[ -L $TMP/root/etc/systemd/system/timers.target.wants/coprocessor-facts.timer ]] ||
    fail "the facts timer isn't enabled"
  assert_mode "$TMP/root/usr/local/lib/coprocessor/coprocessor-facts.sh" 755
}

test_provision_can_be_rerun() {
  setup_provision
  run_provision
  local first
  first=$(tree_digest "$TMP/root")
  run_provision
  assert_eq "$(tree_digest "$TMP/root")" "$first" "the tree after a second run"
}

test_provision_refuses_a_jar_that_isnt_the_locks() {
  setup_provision
  assert_fails "refusing it" run_provision \
    --photonvision-sha256 0000000000000000000000000000000000000000000000000000000000000000
  run_provision --photonvision-sha256 "$(sha256sum <"$TMP/photonvision.jar" | cut -c1-64)"
}

test_provision_refuses_an_agent_unit_that_runs_another_jar() {
  setup_provision
  sed -i 's|/opt/coprocessor/coprocessor-agent.jar|/opt/agent.jar|' "$TMP/coprocessor-agent.service"
  assert_fails "doesn't run /opt/coprocessor/coprocessor-agent.jar" run_provision
}

test_provision_refuses_a_fixed_mac_address() {
  setup_provision
  echo 'ethaddr=02:00:00:00:00:01' >>"$TMP/root/boot/armbianEnv.txt"
  assert_fails "MAC address" run_provision
}

test_provision_refuses_an_image_that_isnt_photonvisions() {
  setup_provision
  rm "$TMP/root/etc/systemd/system/photonvision.service"
  assert_fails "PhotonVision's unit isn't installed" run_provision
}

test_provision_refuses_another_boot_layout() {
  setup_provision
  rm "$TMP/root/boot/boot.scr"
  assert_fails "not the boot layout" run_provision
}

test_provision_refuses_an_image_without_networkmanager() {
  setup_provision
  rm "$TMP/root/usr/sbin/NetworkManager"
  assert_fails "NetworkManager isn't installed" run_provision
}
