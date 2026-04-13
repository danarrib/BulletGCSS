# Self-Hosting a MQTT Broker

## Why Self-Host?

There are several good reasons to run your own MQTT broker instead of relying on a public one:

- **Privacy.** Public brokers offer no privacy guarantees. Anyone who knows or guesses your topic string can subscribe to it and read your telemetry. A private broker with password protection eliminates this risk.
- **Control.** You decide the availability, reliability, and access rules. Public brokers can go down or change their policies without warning.
- **Testing.** A local broker is often more convenient for bench testing and development.

There are some trade-offs:

- You are responsible for administration and uptime.
- You need a server with a public IP address that accepts inbound connections.
- You should obtain a TLS certificate (e.g. from [Let's Encrypt](https://letsencrypt.org/)) to encrypt connections.

---

## Broker Software

This guide covers [Mosquitto](https://mosquitto.org/), a widely used open source MQTT broker that supports MQTT 3.1, 3.1.1, and 5.0. It is lightweight enough to run on a Raspberry Pi or a small VPS.

Mosquitto is available for all major platforms. Linux users should install via their package manager; macOS users via `homebrew`. Binaries for other platforms are available on the [Mosquitto download page](https://mosquitto.org/download/).

The examples below assume a Debian or Ubuntu system. Configuration concepts apply to all platforms; only the installation command and file paths differ.

---

## Installation and Configuration

All steps require root privileges. Prefix commands with `sudo` or open a root shell with `sudo -i`.

### Install Mosquitto

```
# apt update && apt install mosquitto
```

This installs the broker and creates the configuration directory at `/etc/mosquitto`:

```
# ls -l /etc/mosquitto/
total 16
-rw-r--r-- 1 root      root      230 Feb  4 10:14 aclfile.example
drwxr-xr-x 1 root      root       24 Feb 25 11:45 ca_certificates
drwx------ 1 mosquitto mosquitto  52 Feb 25 11:44 certs
drwxr-xr-x 1 root      root      224 Feb 25 11:55 conf.d
-rw-r--r-- 1 root      root      354 Feb  9 09:31 mosquitto.conf
-rw-r--r-- 1 root      root       23 Feb  4 10:14 pskfile.example
-rw-r--r-- 1 root      root      355 Feb  4 10:14 pwfile.example
```

Key directories:
- `certs/` — holds the TLS server certificate and key; readable only by the `mosquitto` user.
- `ca_certificates/` — only needed if you require client certificate validation.
- `conf.d/` — place your local configuration files here.

Do not modify `mosquitto.conf` directly. Add configuration files under `conf.d/` instead.

#### `mosquitto.conf`

The default Debian/Ubuntu file already includes `conf.d`:

```
pid_file /run/mosquitto/mosquitto.pid

persistence true
persistence_location /var/lib/mosquitto/

log_dest file /var/log/mosquitto/mosquitto.log

include_dir /etc/mosquitto/conf.d
```

### Configuration Files

Create the following files under `/etc/mosquitto/conf.d/`:

```
00_general.conf     — global settings
10_mqtt.conf        — plain MQTT listener
20_websocket.conf   — plain WebSocket listener
30_ssl.conf         — MQTT over TLS
40_wss.conf         — WebSocket over TLS
```

#### `00_general.conf`

```
allow_anonymous true
```

Required for Mosquitto 2.0 and newer. Change to `false` once you add password authentication.

#### `10_mqtt.conf`

```
listener 1883
```

Plain MQTT on the standard port. Used for local testing only — do not expose this port to the internet.

#### `20_websocket.conf`

```
listener 8080
protocol websockets
```

Plain WebSocket. For local testing only.

#### `30_ssl.conf`

```
listener 8883
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
```

Encrypted MQTT on the standard TLS port. **This is the port the Bullet GCSS modem connects to.**

#### `40_wss.conf`

```
listener 8081
protocol websockets
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
```

Encrypted WebSocket. **This is the port the Bullet GCSS UI connects to.**

> **Note:** The modem and the UI connect on different ports because they use different protocols: the modem uses a native MQTT/TLS socket (port 8883) and the browser uses a WebSocket connection (port 8081). Both are encrypted. Make sure both ports are open in your firewall.

### Restart and Test

```
# systemctl restart mosquitto
```

Verify that all four ports are listening:

```
# ss -pants | grep mosquitto
LISTEN  0  100   [::]:1883   [::]:*   users:(("mosquitto",pid=2910,fd=6))
LISTEN  0  4096    *:8080      *:*    users:(("mosquitto",pid=2910,fd=9))
LISTEN  0  4096    *:8081      *:*    users:(("mosquitto",pid=2910,fd=14))
LISTEN  0  100   [::]:8883   [::]:*   users:(("mosquitto",pid=2910,fd=11))
```

You can test basic publish/subscribe using the `mosquitto_clients` package:

```
# Subscribe to a topic (run in one terminal):
$ mosquitto_sub -L mqtt://localhost/test/topic

# Publish to the same topic (run in another terminal):
$ mosquitto_pub -L mqtt://localhost/test/topic -m "test message"
```

Use `mqtts://` and port 8883 for TLS tests. Note that the `mosquitto_clients` tools are not tolerant of self-signed certificates; for WebSocket testing or self-signed certificate scenarios, a tool such as [MQTT Explorer](https://mqtt-explorer.com/) is more convenient.

### TLS / SSL Certificates

For a public-facing server, use [Let's Encrypt](https://letsencrypt.org/) certificates. Since the certificate files must be readable only by the `mosquitto` user, copy them into the `certs` directory:

```
certfile /etc/mosquitto/certs/fullchain.pem
keyfile /etc/mosquitto/certs/privkey.pem
```

For internal bench testing, self-signed certificates are sufficient. Browsers will warn about self-signed certificates on the WebSocket connection — to avoid this, generate your own CA certificate and add it to the system trust store on your client devices. There are many `openssl` guides available online for this.

---

## Passwords and Access Control Lists (ACLs)

If your broker is exposed to the internet, you should restrict access with a password file, an ACL, or both.

### Passwords

Use `mosquitto_passwd` to manage the password file. Generate the file in plain text and then encrypt it:

```
# cat /etc/mosquitto/pwfile
mimi:frozenpaw

# mosquitto_passwd -U /etc/mosquitto/pwfile

# cat /etc/mosquitto/pwfile
mimi:$7$101$AA10ftcEp9thwEDV$Nt8...
```

With `allow_anonymous false` in `00_general.conf`, only `mimi` can connect.

### ACLs

An ACL file gives fine-grained control over who can read and write which topics:

```
# cat /etc/mosquitto/aclfile
# Anonymous users can only read the 'test' topic
topic read test

# mimi has full read/write access to all topics
user mimi
topic readwrite #
```

### Applying Password and ACL Settings

Update `00_general.conf` to reference both files:

```
allow_anonymous false
password_file /etc/mosquitto/pwfile
acl_file /etc/mosquitto/aclfile
```

Restart Mosquitto after any configuration change:

```
# systemctl restart mosquitto
```

---

## Connecting Bullet GCSS to Your Broker

Once the broker is running, update the Bullet GCSS settings to point to your server.

**Modem (`Config.h`):**

| Setting | Value |
|---|---|
| Host | your server's hostname or IP |
| Port | `8883` |
| Username | your chosen username |
| Password | your chosen password |
| Topic | `bulletgcss/telem/<your-aircraft-name>` |

**UI (Settings → Broker settings):**

| Setting | Value |
|---|---|
| Host | your server's hostname or IP |
| Port | `8081` |
| Username | your chosen username |
| Password | your chosen password |
| Topic | same topic used on the modem |
| Use TLS | Yes |
