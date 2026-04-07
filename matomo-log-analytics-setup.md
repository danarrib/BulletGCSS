# Matomo Log Analytics Setup Briefing

This document describes the Matomo self-hosted analytics implementation used on **ev.outros.net**,
which runs on a VPS (Ubuntu Server) with DreamHost-managed MySQL.  
The approach works on **any shared or VPS hosting where Apache access logs are available** — no JS
tracking snippet is needed on public pages.

---

## Overview

Traffic data flows from Apache access logs into Matomo via the official `import_logs.py` script,
run nightly via cron. Matomo itself is installed as a standard PHP web app in a subdirectory of
the public site.

```
Apache writes access logs  →  cron runs import_logs.py at 2 AM  →  Matomo DB
                                                                        ↓
                                              cron runs core:archive at 2:30 AM  →  reports ready
```

---

## Prerequisites

- **PHP 8.x** on the server (CLI + web)
- **MySQL database** — DreamHost provides one per panel; Matomo needs its own DB
- **Apache access logs** readable by your shell user, typically at  
  `~/logs/<domain>/https/access.log.YYYY-MM-DD[.gz]` (DreamHost format)  
  or `/var/log/apache2/access.log` (standard Ubuntu VPS)
- **Python 3** for `import_logs.py`

---

## Step 1 — Create a MySQL database for Matomo

On DreamHost: Panel → MySQL Databases → Create a new database and a dedicated user with full
privileges. Note the hostname, username, password, and database name.

---

## Step 2 — Install Matomo

Download the latest Matomo zip from matomo.org and extract it into a **subdirectory** of your
web root, e.g. `~/yourdomain.com/matomo/`.

```bash
cd ~/yourdomain.com
wget https://builds.matomo.org/matomo.zip
unzip matomo.zip
rm matomo.zip
```

Visit `https://yourdomain.com/matomo/` in a browser and follow the installation wizard:
- Enter the DB credentials from Step 1
- Create the first admin account
- Add your site — note the **Site ID** (usually `1`) and the **Auth Token** generated for the admin user

The auth token is used by the log importer. Generate it in:  
Matomo → Administration → Personal → Security → Auth Tokens → Create new token

---

## Step 3 — Confirm the log importer is bundled

Matomo ships `import_logs.py` at:
```
~/yourdomain.com/matomo/misc/log-analytics/import_logs.py
```

No separate download needed.

---

## Step 4 — Write the import script

Create `~/import-stats.sh`:

```bash
#!/bin/bash
# Daily Matomo log importer for yourdomain.com
set -euo pipefail

LOG_DIR="/home/yourusername/logs/yourdomain.com/https"   # adjust for your host
IMPORT_SCRIPT="/home/yourusername/yourdomain.com/matomo/misc/log-analytics/import_logs.py"
MATOMO_URL="https://yourdomain.com/matomo"
TOKEN="<your-auth-token>"
SITE_ID="1"

import_date() {
    local date="$1"
    local plain="${LOG_DIR}/access.log.${date}"
    local gz="${LOG_DIR}/access.log.${date}.gz"

    if [ -f "$plain" ]; then
        LOG_FILE="$plain"
    elif [ -f "$gz" ]; then
        LOG_FILE="$gz"
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: No log found for ${date}, skipping."
        return
    fi

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Importing ${LOG_FILE}..."
    python3 "$IMPORT_SCRIPT" \
        --url="$MATOMO_URL" \
        --token-auth="$TOKEN" \
        --idsite="$SITE_ID" \
        --recorders=2 \
        "$LOG_FILE"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Done."
}

# If a date argument is given, import that specific date; otherwise import yesterday.
if [ $# -gt 0 ]; then
    import_date "$1"
else
    import_date "$(date -d yesterday +%Y-%m-%d)"
fi
```

Make it executable:
```bash
chmod +x ~/import-stats.sh
```

**Log path notes:**
- DreamHost stores Apache logs at `~/logs/<domain>/https/` for HTTPS traffic  
  (separate `http/` directory for plain HTTP, but HTTPS is what matters)
- Log files rotate daily, named `access.log.YYYY-MM-DD` (recent) or `access.log.YYYY-MM-DD.gz` (older)
- The script handles both plain and gzipped files automatically

---

## Step 5 — Test the import manually

Run for a specific past date to verify everything works before setting up cron:

```bash
~/import-stats.sh 2026-04-01
```

Visit Matomo and confirm visits appear under that date.

---

## Step 6 — Set up cron jobs

Add to crontab (`crontab -e`):

```cron
# Matomo: import previous day logs (after midnight log rotation)
0 2 * * * /home/yourusername/import-stats.sh >> /home/yourusername/import-stats.log 2>&1

# Matomo: pre-compute reports (run after the import finishes)
30 2 * * * /usr/bin/php /home/yourusername/yourdomain.com/matomo/console core:archive --url=https://yourdomain.com/matomo >> /home/yourusername/matomo-archive.log 2>&1
```

**Why both jobs:**
- `import-stats.sh` — feeds raw visit data from the log file into Matomo's DB
- `core:archive` — aggregates raw data into reports; without this, the dashboard will be empty  
  (Matomo normally does this on-demand via browser visits, but that requires your cron or a real visit)

**Timing:** run the import at 2 AM because DreamHost rotates logs at midnight; 30-minute gap gives
the import time to finish before archiving starts.

---

## Step 7 — Disable browser-triggered archiving (recommended for cron-only setup)

In Matomo → Administration → General Settings → Archiving:  
Set "Archive reports when viewed from the browser" to **No**.  
This forces all archiving through cron instead of on-demand page loads.

---

## Step 8 — No JS tracking needed

Because the log importer reads raw Apache access logs, you do **not** need to add any JavaScript
tracking snippet to your website's HTML. This means:
- Zero performance impact on page loads
- Works for users with JS disabled or ad blockers
- Static HTML pages (pre-generated, no PHP) are tracked identically to dynamic pages

---

## Apache log format

The default Apache Combined Log Format is recognized by `import_logs.py` automatically:

```
87.250.224.12 - - [01/Apr/2026:01:02:55 -0700] "GET /path HTTP/1.1" 200 16051 "-" "Mozilla/5.0..."
```

No custom log format configuration is needed.

---

## Matomo config.ini.php reference

After installation, Matomo stores its config at `matomo/config/config.ini.php`. The key sections:

```ini
[database]
host = "mysql.dreamhost.com"    ; your DreamHost MySQL hostname
username = "matomo_user"
password = "yourpassword"
dbname = "matomo_yourdb"
tables_prefix = "matomo_"
charset = "utf8mb4"
collation = "utf8mb4_0900_ai_ci"

[General]
trusted_hosts[] = "yourdomain.com"
```

---

## Importing historical data

To backfill data from before Matomo was set up, run the script for each past date:

```bash
# Import a range of dates
for d in $(seq 30 -1 1); do
    ~/import-stats.sh "$(date -d "$d days ago" +%Y-%m-%d)"
done
```

Then run `core:archive` manually once:
```bash
php ~/yourdomain.com/matomo/console core:archive --url=https://yourdomain.com/matomo
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Dashboard shows no data | Archiving not run | Run `core:archive` manually |
| `import_logs.py` errors on auth | Wrong token or site ID | Regenerate token in Matomo panel |
| Logs not found | Wrong `LOG_DIR` path | Check `ls ~/logs/` for actual path |
| Visits doubled | Ran import twice for same date | Matomo deduplicates by request time — usually safe |
| `python3` not found | Missing Python 3 | Install with `apt install python3` (VPS) or check hosting panel |

---

## File summary

| File | Purpose |
|---|---|
| `~/yourdomain.com/matomo/` | Matomo web app (downloaded from matomo.org) |
| `~/yourdomain.com/matomo/misc/log-analytics/import_logs.py` | Bundled log importer |
| `~/import-stats.sh` | Wrapper script: imports yesterday's logs |
| `~/import-stats.log` | Output log for the import cron job |
| `~/matomo-archive.log` | Output log for the archiving cron job |
| crontab | Two entries: import at 2 AM, archive at 2:30 AM |
