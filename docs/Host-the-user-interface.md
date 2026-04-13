# Hosting the User Interface

The Bullet GCSS UI is a web app and needs to be served over HTTP/HTTPS to be used.

---

## Short Answer — You Don't Have To

The UI is already hosted at **[https://bulletgcss.outros.net/](https://bulletgcss.outros.net/)** and is free to use. It is updated automatically every time the `master` branch is pushed.

---

## Long Answer — Self-Hosting

### What the UI is made of

The UI is a static web application: one HTML file, a handful of JavaScript modules, CSS, and image assets. There is no backend server required.

The one exception is **`proxy.php`** — a small reverse proxy used for the terrain elevation feature in the Mission Planner. It forwards requests to `api.open-elevation.com` to avoid browser CORS restrictions. If your hosting environment does not support PHP, Bullet GCSS will work fully except for terrain elevation data on waypoints.

### Hosting options

**GitHub Pages** is the simplest option if you want your own deployment. Fork the repository, enable GitHub Pages on the `master` branch, and point it at the `UI/` directory. No configuration needed.

**Any static web host** (shared hosting, VPS, S3, Netlify, Cloudflare Pages, etc.) will work. Copy the contents of the `UI/` directory to your web root and open `basicui.html`. You may want to configure your web server to serve `basicui.html` as the default index page so you don't have to type the filename in the URL.

### HTTPS requirement

If your MQTT broker uses TLS for WebSocket connections (which it should), the UI **must** be served over HTTPS. Most modern hosting providers offer free TLS certificates. A plain HTTP host will cause the browser to block the secure WebSocket connection.

### Configuring `proxy.php`

The `proxy.php` file has a security allowlist that restricts which external domains it will forward requests to. The default allowlist contains only `api.open-elevation.com` and should not be changed.

> **Note:** Older versions of this file also contained `bulletgcss.outros.net` in the allowlist. If you copied `proxy.php` from an older version, remove that entry — it is not needed.
