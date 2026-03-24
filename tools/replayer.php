<?php
/**
 * BulletGCSS Flight Replayer
 *
 * Replays a saved BulletGCSS flight log to an MQTT broker, preserving the
 * original message timing. Designed to run as a PHP CLI script via cron.
 *
 * Usage:
 *   php replayer.php
 *
 */

// ─── Configuration ────────────────────────────────────────────────────────────

define('MQTT_HOST',      'broker.emqx.io');
define('MQTT_PORT',      1883);           // 1883 = plain TCP, 8883 = TLS
define('MQTT_USE_TLS',   false);
define('MQTT_USERNAME',  'bulletgcss');
define('MQTT_PASSWORD',  'bulletgcss');
define('MQTT_TOPIC',     'revspace/sensors/dnrbtelem');
define('MQTT_CLIENT_ID', 'replayer_' . substr(md5(uniqid()), 0, 8));

define('LOG_FILE',       __DIR__ . '/testflight2.txt');

// ─────────────────────────────────────────────────────────────────────────────


function mqtt_encode_length(int $length): string
{
    $result = '';
    do {
        $byte = $length % 128;
        $length = (int)($length / 128);
        if ($length > 0) $byte |= 0x80;
        $result .= chr($byte);
    } while ($length > 0);
    return $result;
}

function mqtt_connect(string $host, int $port, bool $useTLS, string $clientId, string $username, string $password): mixed
{
    $address = ($useTLS ? 'tls://' : 'tcp://') . $host . ':' . $port;
    $socket = stream_socket_client($address, $errno, $errstr, 10);
    if (!$socket) {
        throw new RuntimeException("Connection failed: $errstr ($errno)");
    }
    stream_set_timeout($socket, 5);

    // Variable header
    $varHeader  = "\x00\x04MQTT"; // Protocol name
    $varHeader .= "\x04";         // Protocol level: 4 = MQTT 3.1.1

    $flags = 0x02; // Clean session
    if ($username !== '') $flags |= 0x80;
    if ($password !== '') $flags |= 0x40;
    $varHeader .= chr($flags);
    $varHeader .= "\x00\x3C"; // Keep-alive: 60 s

    // Payload
    $payload  = pack('n', strlen($clientId)) . $clientId;
    if ($username !== '') $payload .= pack('n', strlen($username)) . $username;
    if ($password !== '') $payload .= pack('n', strlen($password)) . $password;

    $remaining = $varHeader . $payload;
    $packet = "\x10" . mqtt_encode_length(strlen($remaining)) . $remaining;
    fwrite($socket, $packet);

    // Read CONNACK (4 bytes)
    $response = fread($socket, 4);
    if (strlen($response) < 4 || ord($response[0]) !== 0x20) {
        throw new RuntimeException("Did not receive CONNACK");
    }
    $returnCode = ord($response[3]);
    if ($returnCode !== 0) {
        throw new RuntimeException("CONNACK error code: $returnCode");
    }

    return $socket;
}

function mqtt_publish(mixed $socket, string $topic, string $message): void
{
    // Fixed header: type=3 (PUBLISH), QoS 0, no retain, no dup
    $topicPart = pack('n', strlen($topic)) . $topic;
    $payload   = $topicPart . $message;
    $packet    = "\x30" . mqtt_encode_length(strlen($payload)) . $payload;
    fwrite($socket, $packet);
}

function mqtt_disconnect(mixed $socket): void
{
    fwrite($socket, "\xe0\x00");
    fclose($socket);
}


// ─── Load and parse log file ──────────────────────────────────────────────────

$lines = file(LOG_FILE, FILE_IGNORE_NEW_LINES | FILE_SKIP_EMPTY_LINES);
if ($lines === false) {
    die("Cannot read log file: " . LOG_FILE . "\n");
}

$messages = [];
foreach ($lines as $line) {
    $parts = explode('|', $line, 2);
    if (count($parts) !== 2) continue;
    [$ts, $payload] = $parts;
    if (str_starts_with($payload, 'Connected to')) continue;
    $messages[] = [(int)$ts, $payload];
}

if (count($messages) === 0) {
    die("No messages found in log file.\n");
}

// Detect gaps larger than 1 minute between consecutive messages
$gapThresholdMs = 60000;
for ($i = 1; $i < count($messages); $i++) {
    $gapMs = $messages[$i][0] - $messages[$i - 1][0];
    if ($gapMs >= $gapThresholdMs) {
        $gapMin = (int)($gapMs / 60000);
        $gapSec = (int)(($gapMs % 60000) / 1000);
        $startTs = $messages[$i - 1][0];
        $endTs   = $messages[$i][0];
        echo "Gap of {$gapMin}m {$gapSec}s starting at {$startTs} ending at {$endTs}\n";
    }
}

$totalMs  = $messages[count($messages) - 1][0] - $messages[0][0];
$totalMin = (int)($totalMs / 60000);
$totalSec = (int)(($totalMs % 60000) / 1000);

echo date('Y-m-d H:i:s') . " Starting replay of " . count($messages) . " messages to " . MQTT_TOPIC . "\n";
echo date('Y-m-d H:i:s') . " Session duration: {$totalMin}m {$totalSec}s\n";


// ─── Connect ──────────────────────────────────────────────────────────────────

try {
    $socket = mqtt_connect(MQTT_HOST, MQTT_PORT, MQTT_USE_TLS, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
} catch (RuntimeException $e) {
    die("MQTT error: " . $e->getMessage() . "\n");
}

echo date('Y-m-d H:i:s') . " Connected to " . MQTT_HOST . ":" . MQTT_PORT . "\n";


// ─── Replay with original timing ──────────────────────────────────────────────

$firstTs   = $messages[0][0];
$startTime = microtime(true);

foreach ($messages as [$ts, $payload]) {
    // Time (ms) this message was sent relative to the start of the flight
    $targetMs = $ts - $firstTs;
    // Time (ms) elapsed since we started replaying
    $elapsedMs = (int)((microtime(true) - $startTime) * 1000);
    // Sleep until it is time to send this message
    $sleepUs = ($targetMs - $elapsedMs) * 1000;
    if ($sleepUs > 0) {
        usleep((int)$sleepUs);
    }

    mqtt_publish($socket, MQTT_TOPIC, $payload);
}

mqtt_disconnect($socket);
echo date('Y-m-d H:i:s') . " Replay complete.\n";
