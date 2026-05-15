#!/bin/sh
set -e

update_config_field() {
    if grep -qE "^\[$2\]" "$1" && grep -qE "^$3\s*=" "$1"; then
        val=$(grep -E "^\[$2\]" -A 1000 "$1" | grep -m 1 -E "^$3\s*=" | sed -E 's/^[^=]*=\s*//')
        sed "/^\[$5]/,/^\[/{s/^$6[[:space:]]*=.*/$6 = $val/}" "$4" > "$4.tmp" && cp "$4.tmp" "$4" && rm "$4.tmp"
    else
        echo "Field '$3' in section '$2' not found in '$1'. Skipping update."
    fi
}

if [ -e "/info.ini" ]; then
    update_config_field "/info.ini" "general" "id" "/config.ini" "station" "id"
    update_config_field "/info.ini" "mobility" "stationType" "/config.ini" "station" "type"
    update_config_field "/info.ini" "mobility" "latitude" "/config.ini" "station" "latitude"
    update_config_field "/info.ini" "mobility" "longitude" "/config.ini" "station" "longitude"
    update_config_field "/info.ini" "mobility" "macAddr" "/config.ini" "station" "mac_address"
    update_config_field "/info.ini" "mobility" "interface" "/config.ini" "general" "interface"
    update_config_field "/info.ini" "general" "id" "/config.ini" "general" "dds_domain_id"
    echo "Config update process complete"
else
    echo "No global board config file found. Skipping config update process"
fi

IP_ADDR=$(ip -f inet addr show eth0 | awk '/inet / {print $2}')
GW_ADDR=$(ip r | awk '/default / {print $3}')
BR_ID=br0

if [ -n "$SUPPORT_MAC_BLOCKING" ] && [ "$SUPPORT_MAC_BLOCKING" = "true" ]; then
    brctl addbr $BR_ID
    ip a a $IP_ADDR dev $BR_ID
    ip a d $IP_ADDR dev eth0
    brctl addif $BR_ID eth0
    ip link set $BR_ID up
    ip r a default via $GW_ADDR
fi

printf '#!/bin/sh\nebtables -A INPUT -s $1 -j DROP;' > /bin/block
printf '#!/bin/sh\nebtables -D INPUT -s $1 -j DROP;' > /bin/unblock
chmod +x /bin/block
chmod +x /bin/unblock

LOCAL_MQTT_PORT="${EMBEDDED_MOSQUITTO_PORT:-1883}"
REMOTE_MQTT_HOST="${REMOTE_MQTT_HOST:-mqtt-broker}"
REMOTE_MQTT_PORT="${REMOTE_MQTT_PORT:-1883}"
VEHICLE_ID="${VEHICLE_ID:-vehicle_1}"

if [ -n "$START_EMBEDDED_MOSQUITTO" ] && [ "$START_EMBEDDED_MOSQUITTO" = "true" ]; then
    {
        echo "listener ${LOCAL_MQTT_PORT} 0.0.0.0"
        echo "allow_anonymous true"
        if [ -n "$REMOTE_MQTT_HOST" ]; then
            echo "connection bridge_to_remote"
            echo "address ${REMOTE_MQTT_HOST}:${REMOTE_MQTT_PORT}"
            echo "topic car/${VEHICLE_ID}/sensors/# in 0"
            echo "topic car/${VEHICLE_ID}/actuators/# out 0"
            echo "topic car/${VEHICLE_ID}/status/# out 0"
            if [ "${MIRROR_VANETZA_TOPICS:-true}" = "true" ]; then
                echo "topic vanetza/# out 0 \"\" \"obu/${VEHICLE_ID}/\""
            fi
        fi
    } > /mosquitto.conf
    /usr/sbin/mosquitto -c /mosquitto.conf &
    sleep 1
fi

/usr/local/bin/socktap -c /config.ini &

exec python3 /app/main.py
