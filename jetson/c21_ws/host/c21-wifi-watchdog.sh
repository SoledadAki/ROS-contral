#!/bin/bash
# Reconnect the active WiFi connection when wlan0 is associated but the network is dead (seen after AP roaming:
# associated, but the gateway never answers ARP). Run as root every 30 s by c21-wifi-watchdog.timer.
#   sudo install -m755 c21-wifi-watchdog.sh /usr/local/bin/
#   sudo cp c21-wifi-watchdog.{service,timer} /etc/systemd/system/
#   sudo systemctl daemon-reload && sudo systemctl enable --now c21-wifi-watchdog.timer

DEV=wlan0
STATE=/run/c21-wifi-watchdog
COOLDOWN=120   # s after a reconnect before trying again

[ "$(nmcli -t -g GENERAL.STATE dev show $DEV 2>/dev/null)" = "100 (connected)" ] || exit 0
CON=$(nmcli -t -g GENERAL.CONNECTION dev show $DEV)   # whatever wlan0 is on (campus WiFi or a hotspot)
gw=$(ip -4 route show default dev $DEV | awk '{print $3; exit}')
if { [ -n "$gw" ] && ping -c3 -W1 -I $DEV "$gw" >/dev/null 2>&1; } || ping -c2 -W2 -I $DEV 223.5.5.5 >/dev/null 2>&1; then
    rm -f $STATE.fail
    exit 0
fi
n=$(( $(cat $STATE.fail 2>/dev/null || echo 0) + 1 ))
echo $n > $STATE.fail
[ $n -ge 2 ] || exit 0                                   # two checks in a row (~30 s)
last=$(cat $STATE.last 2>/dev/null || echo 0)
[ $(( $(date +%s) - last )) -ge $COOLDOWN ] || exit 0
date +%s > $STATE.last
logger -t c21-wifi-watchdog "gateway ${gw:-?} unreachable on $DEV, reconnecting $CON"
nmcli con up "$CON" >/dev/null 2>&1
logger -t c21-wifi-watchdog "now $(ip -4 -br a show $DEV | awk '{print $3}')"
rm -f $STATE.fail
