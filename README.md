# SZF Bots

A basic bot support plugin for the [Super Zombie Fortress](https://github.com/redsunservers/SuperZombieFortress) gamemode using [KittyRifter's navmesh plugin](https://github.com/KitRifty/sourcepawn-navmesh/tree/master) to pathfind.

Convars
-------
 - `zbot_valid_paths_only <1/0>` Take fully connected paths only
 - `zbot_max_hardstuck_time <#>` Time in seconds till teleporting to nearest zombie player when totally stuck
 - `zbot_recalc_on_node <1/0>` Recalculate path immediately when enemy enters a new nav area

Installation
-------
Compile the navmesh.sp and zbot.sp files, and move them into your server's `addons/sourcemod/plugins` folder. The map must have an associated navmesh for the bots to function.
