## Introduction

This article discusses the configuration and usage of a self-hosted MQTT broker. It was originally hosted in the [fl2mqtt wiki](https://github.com/stronnag/bbl2kml/wiki) but it's much more relevant to BulletGCSS. 

There are a number of reasons for considering using a self-hosted broker:

 * Privacy. While there is no reason to believe that the public brokers would retain your data, there is no guarantee of privacy.
    - Choosing a good topic provides some protection against a third-party snooping your data; however some brokers provide wildcard topics, so privacy via obscurity is not a panacea.
	- The free brokers usually do not offer password protection for topics, anyone who can guess a topic can see your data.
* Convenience. Particularly for testing and development, it may be more convenient to run a local broker.
* Control. You control the reliability and availability of the broker. The public brokers are for the convenience / benefit of the operator and may be unavailable without warning.

There are of course some downsides to self-hosting:

* You are the administrator and single point of failure

* You may need to acquire new skills to configure the broker

* You need a public IP address that accepts inbound connections

* You need some hardware to run the (very low resource) server, or use a cloud provider.

* In order to protect data in flight, you should acquire your own TLS / SSL  certificate, e.g. from [Let's Encrypt](https://letsencrypt.org/).

However, you're now in control of your privacy and security, and can use encryption and password access controls, without any third party potentially having access to your plain-text data.

## Broker Software

The rest of the article discusses the installation, configuration and usage of the [mosquitto](https://mosquitto.org/) MQTT broker. [Mosquitto](https://mosquitto.org/) describes itself as:

>Eclipse Mosquitto is an open source (EPL/EDL licensed) message broker that implements the MQTT protocol versions 5.0, 3.1.1 and 3.1. Mosquitto is lightweight and is suitable for use on all devices from low power single board computers to full servers.

>The MQTT protocol provides a lightweight method of carrying out messaging using a publish/subscribe model. This makes it suitable for Internet of Things messaging such as with low power sensors or mobile devices such as phones, embedded computers or micro-controllers.

[Mosquitto](https://mosquitto.org/) is available for all common operating systems (most Linux distributions, MacOS, Microsoft Windows, FreeBSD etc.). [Mosquitto](https://mosquitto.org/) provides binaries from [their download page](https://mosquitto.org/download/).  Linux / *BSD users should use their distro installation tools (apt, dnf, pacman, pkg etc.). On MacOS, it is recommended to use `homebrew`.

The examples that follow assume a Debian or derivative (Ubuntu, Raspbian et al). Other than the initial installation and file locations, the configuration steps will be common to all.

## Installation and configuration

All the steps are run from a terminal and will need root privileges, you can either prefix the steps with `sudo` or enter a root shell `sudo -i`.

### Install Mosquitto

```
## optional, ensure system is upto-date
# apt update && apt upgrade
## install msoquitto
# apt install mosquitto
```

This will install the application and required files. The configuration is in the directory `/etc/mosquitto`.

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

Note:

* The `ca_certificates` only needs to be populated if you're using client certificate validation.

* The `certs` directory holds the TLS/SSL server certificate and key. This should be readable only by the `mosquitto` user.

* The `conf.d` directory holds configuration files. You should not add local configuration to the distributed `moquitto.conf` (other than to add an `include_dir` directive for `conf.d` if necessary).

* The `*.example` files give examples for additional security features, beyond the scope of this basic guide.

* In order to follow the rest of this guide, it's assumed the installation provided or you created the `conf.d`, `certs` and `ca_certificates` directories as necessary.

Each of local configuration files is described in more detail below.

#### `mosquitto.conf`

The Debian / Ubuntu file is minimalist:

```
# cat /etc/mosquitto/mosquitto.conf
# Place your local configuration in /etc/mosquitto/conf.d/
#
# A full description of the configuration file is at
# /usr/share/doc/mosquitto/examples/mosquitto.conf.example

pid_file /run/mosquitto/mosquitto.pid

persistence true
persistence_location /var/lib/mosquitto/

log_dest file /var/log/mosquitto/mosquitto.log

include_dir /etc/mosquitto/conf.d
```

We shall follow the advice and place our configuration files in the `conf.d` directory, here's what are added (and described below):

```
# ls -l conf.d
total 24
-rw-r--r-- 1 root root  21 Feb 25 11:55 00_general.conf
-rw-r--r-- 1 root root  14 Feb 24 11:03 10_mqtt.conf
-rw-r--r-- 1 root root  34 Feb  3 19:06 20_websocket.conf
-rw-r--r-- 1 root root 140 Feb 25 11:53 30_ssl.conf
-rw-r--r-- 1 root root 162 Feb 25 11:51 40_wss.conf
-rw-r--r-- 1 root root 142 Feb  9 09:31 README
```

Five files have been added:

* `00_general.conf` : General settings, perhaps access / security related.
* `10_mqtt.conf` : Configure MQTT protocol, not encrypted.
* `20_websocket.conf` : Configure MQTT over websockets, not encrypted.
* `30_ssl.conf` : Configure MQTT protocol, TLS/SSL encrypted.
* `40_websocket.conf` : Configure MQTT over websockets, TLS/SSL encrypted.

These files represent a basic configuration. If you need more help on advanced topics (generating security certificates, password files, limiting access by some criteria), please try [Google](https://www.google.com/) or ask in the [Telegram channel](https://t.me/bulletgcss) or [RC Groups forum](https://www.rcgroups.com/forums/showthread.php?3804715-Bullet-GCSS-Semi-asynchronous-internet-based-ground-control-station-system).

In the following examples, the service port allocation follows the convention used by the public test broker `test.mosquitto.org`.

#### `00_general.conf`

For the most basic setting, this file only needs one setting (for versions prior to mosquitto 2.0, this is not required, but harmless).

```
# cat conf.d/00_general.conf
allow_anonymous true
```

#### `10_mqtt.conf`

This file defines the port for the MQTT listener; the default port is used (1883).

```
# cat conf.d/10_mqtt.conf
listener 1883
```

#### `20_websockets.conf`

This file defines the port and protocol for non-encrypted websockets.  Port (8080) is your choice.

```
# cat conf.d/20_websocket.conf
listener 8080
protocol websockets
```

#### `30_ssl.conf`

This file defines the port and protocol for encrypted MQTT.  Port is your choice, 8883 is widely used. This example uses a self-signed SSL certificate; other certificate options are discussed below.

```
# cat conf.d/30_ssl.conf
listener 8883
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
```

#### `40_ssl.conf`

This file defines the port and protocol for encrypted MQTT over websockets.  Port (8081) is your choice. This example uses a self-signed SSL certificate; other certificate options are discussed below.

```
# cat conf.d/40_wss.conf
listener 8081
protocol websockets

certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
```

**Note: The MQTT protocol (as used by the air side to publish data) and the Websockets protocol (as used the the Bullet GCSS web application) use different ports (and TLS adds another port to both). It is important to understand this difference and configure both MQTT and Websocket ports.**

### Restart and test

You can now restart the MQTT server.

```
## Systemd management
# systemctl restart mosquitto

## or, legacy Debian / Ubuntu management
# service mosquitto restart
```

You should be able to see the ports in use:

```
# ss -pants
State            Recv-Q           Send-Q                                            Local Address:Port                                                Peer Address:Port            Process
...
LISTEN           0                100                                                        [::]:1883                                                        [::]:*                users:(("mosquitto",pid=2910,fd=6))
LISTEN           0                4096                                                          *:8080                                                           *:*                users:(("mosquitto",pid=2910,fd=9))
LISTEN           0                4096                                                          *:8081                                                           *:*                users:(("mosquitto",pid=2910,fd=14))
LISTEN           0                100                                                        [::]:8883                                                        [::]:*                users:(("mosquitto",pid=2910,fd=11))
...
```

We can see `mosquitto` listening on the four ports defined in the configuration files.

If you install the `mosquitto_clients` package, you can test basic operation using `mosquitto_sub` and `mosquitto_pub` tools, either in separate terminals, or better, across a number of machines.

```
# Test subscription to "test/topic" on machine "toybox":
$ mosquitto_sub -L mqtt://toybox/test/topic
```

```
# Test publication to "test/topic" on machine "toybox":
$ mosquitto_pub -L mqtt://toybox/test/topic -m "Mossy test"
```

In the `mosquitto_sub` instance, you should see:

```
Mossy test
```
Use a scheme of `mqtts://` and port of 8883 for TLS tests. You can use a MQTT sub and MQTTS pub and vice-versa. Note that the `mosquitto_client` tools are not tolerant of self-signed certificates; the MQTT clients from the [mqtttest](https://github.com/stronnag/mqtttest) repository are more tolerant, and provide specific BulletGCSS support. The  [mqtttest](https://github.com/stronnag/mqtttest) tools can also test the websocket options, which the mosquitto tools cannot do. You can also use [fl2mqtt](https://github.com/stronnag/bbl2kml) for publishing flight logs (Blackbox, OpenTX).

Of course, you can also use the BulletGCSS aircraft device and web application, but it's preferable and easier to test the broker independently first.

Note that if you are using `mosquitto` websockets with Mozilla Firefox, it is advisable to set the Firefox `about:config` variable `network.http.spdy.websockets` to `false`, otherwise you may be plagued by hard to diagnose MQTT connection problems.

### TLS / SSL Options

If you have a public facing server, it is recommended that you use [Let's Encrypt](https://letsencrypt.org/) certificates. Note that as the server certificate and key should only be readable by the `mosquitto` user, it may be necessary to copy the system files into the `certs` directory and adjust permissions accordingly, for example (for both the mqtts: and wss: configurations), using standard Let's Encrypt names:

```
certfile /etc/mosquitto/certs/fullchain.pem
keyfile /etc/mosquitto/certs/privkey.pem
```

For internal testing, self signed certificates are adequate. In order to use the BulletGCSS web application without the browser getting upset with you, it is recommended that you generate your own CA certificate as well as server certificate / key and add the generated CA certificate to the system store on client systems. There are numerous guides on doing this with `openssl` on the internet, or ask in the Telegram channel.

### Passwords and Access Control Lists (ACLs)

Some brief notes on passwords and ACLs. If you're self hosting, and exposed to the internet (vice a private test network), you probably want to add password access (and / or ACLs). There are a number of possibilities:

* You have no access controls; or
* You have password access and no anonymous users; or
* You have (privileged) password access uses and limited anonymous access

We'll discuss them all (did I say brief?).

#### Passwords

The password file is managed by the `mosquitto_passwd` application. The easiest way to generate an *initial* encrypted password file is the generate the file in plain text and use  `mosquitto_passwd -U` to update it to an encrypted file, for example, for the user `mimi` with password `frozenpaw` in a file `pwfile`:

```
# cat pwfile
mimi:frozenpaw
## Now encrypt it
# mosquitto_passwd -U pwfile
# cat pwfile
mimi:$7$101$AA10ftcEp9thwEDV$Nt8qTuEgqu+JXMGe4giy25D3sdql4mnLqHNyDo2lh1g6AHUDVk+Te/Xm7f6keWk7JVOcPh0criEwKb5tpBhoQQ==
```
If we set `allow_anonymous false`, then the only user permitted is `mimi`. If we set `allow_anonymous true`, then anyone can still access our MQTT broker, and giving `mimi` a password was somewhat pointless ... unless we combine it with ACLs.

#### ACLs (Access Control Lists)

We can define access control in an ACL file (here `aclfile`); there are numerous guides on the internet, this is just a quick flyby.
```
# cat aclfile
# Anyone, even anonymous, can read 'test'
topic read test

# Give mimi full access to everything ('#' is the wildcard)
user mimi
topic readwrite #
```
#### Password / ACL configuration

The final state of `conf.d/00_general.conf` looks like:
```
allow_anonymous true
#allow_anonymous false
password_file /etc/mosquitto/pwfile
acl_file /etc/mosquitto/aclfile
```
Anonymous users can access the `test` topic, `mimi` can publish and subscribe to any topic.

 


 

