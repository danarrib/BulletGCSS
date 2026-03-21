Each INAV mission waypoint has it's own "Altitude" value, which indicates the altitude that aircraft should be when reaching the waypoint.
This altitude is always **in relation to the Home point altitude**, and Home point altitude will vary depending on where the home point is set.

![image](https://user-images.githubusercontent.com/17026744/107130645-6b603f00-68ae-11eb-991c-8c900087bf88.png)

Take the above picture as example:
* Home point is at 851 meters above sea level.
* Waypoint 1 is defined to be 50 meters above home point altitude ("WP Alt" value on the picture)
* So, the altitude of the Waypoint 1 in relation to the sea level is 851 + 50, which is 901 meters above sea level.
* The terrain altitude (elevation) on the Waypoint 1 is 848 meters above sea level, which is 3 meters lower than the altitude at the home point.
* So, if the Waypoint altitude in relation to the seal level is 901 meters, and the terrain altitude is 848 meters, then the aircraft will be 53 meters above the ground.

INAV doesn't report the terrain elevation to Bullet GCSS (it doesn't has this information). But thanks to the information kindly provided by **Open Topo Data**, Bullet GCSS can fetch this information, do this calculations and show the altitude in relation to the ground for each waypoint.

Thanks to [Andrew Nisbet](https://www.ajnisbet.com/) from [OpenTopoData.org](https://www.opentopodata.org/) for kindly supporting Bullet GCSS project with this valuable source of information.