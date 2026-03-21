The Bullet GCSS User Interface (UI) needs to be hosted somewhere on the Internet to be used, since it's a Web App.

## How do I host the User Interface?

### Short answer

**You don't have to**. It's already hosted by Outros.net: Just go to [https://bulletgcss.outros.net/](https://bulletgcss.outros.net/) and use it from there. It's free and safe.

![Deploy BulletGCSS UI on FPV Sampa](https://github.com/danarrib/BulletGCSS/workflows/Deploy%20BulletGCSS%20UI%20on%20FPV%20Sampa/badge.svg)

Every time the `master` branch is updated, GitHub publishes automatically on this address. Can't be easier, right?

### Long answer

Bullet GCSS UI is a simple HTML page, with some Javascript files and libraries and some CSS files. It also has some images (mostly icons used on the UI).

**There is only one server side program** on Bullet GCSS UI. It's a reverse proxy for API calls (`proxy.php`) to avoid [CORS](https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS) warnings on the Web Browser. It's used only for the "Terrain elevation" fetch feature of Waypoints mission. If no server-side processing is available, Bullet GCSS will work just fine, only this feature will be missing.

So, basically, **any web hosting** will be able to host Bullet GCSS UI. There are lots of free and paid hosting companies out there for you to choose.

To host it, just copy the entire "UI" directory from this repository to your web hosting environment. Then, open the "basicui.html" file from there.

That's all. Simple, right?

But wait, there's a catch: If you're using a MQTT Broker that uses TLS for WebSockets connection (and you really should use it), then you have to host the UI in a http**s** (secure) environment. Your hosting company probably knows how to do it.