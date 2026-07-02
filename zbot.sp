#include <sourcemod>
#include <sdktools>
#include <tf2>
#include <tf2_stocks>
#include <navmesh>
#include <sdkhooks>

#pragma newdecls required

// Pathfinding
ArrayList g_aPathNodes[MAXPLAYERS + 1];
int       g_iCurrentNode[MAXPLAYERS + 1];
float     g_flNextPathUpdate[MAXPLAYERS + 1];
int       g_iTargetEnemy[MAXPLAYERS + 1];
bool      g_bDidInitialDeath[MAXPLAYERS + 1];
CNavArea  g_hLastEnemyArea[MAXPLAYERS + 1];  

// Aim smoothing
float     g_flAimMomentX[MAXPLAYERS + 1];
float     g_flAimMomentY[MAXPLAYERS + 1];
float     g_flDesiredAngles[MAXPLAYERS + 1][3];

// Obstruction Detection
float     g_flStuckTime[MAXPLAYERS + 1];
float     g_flStuckCheckPos[MAXPLAYERS + 1][3];  

// Stuck Detection
float     g_flLastMoveTime[MAXPLAYERS + 1];
float     g_flLastMovePos[MAXPLAYERS + 1][3];

bool      g_bRoundLive = false;

// ConVars
ConVar    g_cvRecalcOnNode;
ConVar    g_cvMaxHardstuckTime;
ConVar    g_cvValidPathsOnly;
ConVar    g_hNbPlayerStop; 

#define STUCK_RADIUS     10.0  
#define STUCK_WAIT_TIME   0.5  

#define AIM_UPDATE_RANGE     500.0  
#define PATH_UPDATE_RATE       0.2   

public Plugin myinfo =
{
    name        = "[SZF] Zombie Bots",
    author      = "gamerjoee",
    description = "Zombie melee bots for SZF",
    version     = "1.0",
    url         = ""
};

public void OnPluginStart()
{
    for (int i = 1; i <= MaxClients; i++)
    {
        g_aPathNodes[i]     = new ArrayList(3);
        g_iCurrentNode[i]   = -1;
        g_hLastEnemyArea[i] = INVALID_NAV_AREA;
    }

    g_cvRecalcOnNode = CreateConVar("zbot_recalc_on_node", "0", "Recalculate path immediately when enemy enters a new nav area", FCVAR_NONE, true, 0.0, true, 1.0);
    g_cvMaxHardstuckTime = CreateConVar("zbot_max_hardstuck_time", "10.0", "Seconds remaining stuck before teleporting to a human zombie", FCVAR_NONE, true, 0.0);
    g_cvValidPathsOnly = CreateConVar("zbot_valid_paths_only", "0", "Require a fully connected path", FCVAR_NONE, true, 0.0, true, 1.0);

    g_hNbPlayerStop = FindConVar("nb_player_stop"); 
    if (g_hNbPlayerStop != null)
    {
        g_hNbPlayerStop.Flags &= ~FCVAR_CHEAT;
        g_hNbPlayerStop.IntValue = 1;
    }

    HookEntityOutput("func_door", "OnOpen", Event_DoorOpened);
    HookEntityOutput("func_door_rotating", "OnOpen", Event_DoorOpened);
    HookEvent("player_spawn",            Event_PlayerSpawn, EventHookMode_Post);
    HookEvent("teamplay_round_start",    Event_RoundStart,  EventHookMode_Post);
    HookEvent("teamplay_setup_finished", Event_RoundActive, EventHookMode_Post);
}


public void OnClientPutInServer(int client)
{
    g_bDidInitialDeath[client]     = false;
    g_iTargetEnemy[client]         = -1;
    g_iCurrentNode[client]         = -1;
    g_flStuckTime[client]          = 0.0;
    g_hLastEnemyArea[client]       = INVALID_NAV_AREA;
    g_flAimMomentX[client]         = 0.0;
    g_flAimMomentY[client]         = 0.0;
    g_flDesiredAngles[client][0]   = 0.0;
    g_flDesiredAngles[client][1]   = 0.0;
    g_flDesiredAngles[client][2]   = 0.0;
    g_flStuckCheckPos[client][0]   = 0.0;
    g_flStuckCheckPos[client][1]   = 0.0;
    g_flStuckCheckPos[client][2]   = 0.0;
    g_flLastMoveTime[client]       = 0.0;

    g_aPathNodes[client].Clear();
}

public void OnClientDisconnect_Post(int client)
{
    g_aPathNodes[client].Clear();
    g_iCurrentNode[client]   = -1;
    g_iTargetEnemy[client]   = -1;
    g_hLastEnemyArea[client] = INVALID_NAV_AREA;
}


/////////////////
/* Pathfinding */
/////////////////

bool BuildNavPath(int client, float flGoalPos[3])
{
    if (!NavMesh_Exists()) return false;

    float flBotPos[3];
    GetClientAbsOrigin(client, flBotPos);
    flBotPos[2] += 15.0; 

    CNavArea startArea = GetAreaAtFeet(flBotPos, GetClientTeam(client));
    CNavArea goalArea  = GetAreaAtFeet(flGoalPos);

    if (startArea == INVALID_NAV_AREA || goalArea == INVALID_NAV_AREA)
    {
        g_aPathNodes[client].Clear();
        g_iCurrentNode[client] = -1;
        return false;
    }

    CNavArea closestArea  = INVALID_NAV_AREA;
    bool bCompletePath    = NavMesh_BuildPath(startArea, goalArea, flGoalPos, TFPathCost, _, closestArea);

    if (g_cvValidPathsOnly.BoolValue)
    {
        if (closestArea == INVALID_NAV_AREA || closestArea == startArea)
        {
            g_aPathNodes[client].Clear();
            g_iCurrentNode[client] = -1;
            return false;
        }
    }

    g_aPathNodes[client].Clear();

    g_aPathNodes[client].PushArray(flGoalPos, 3);

    CNavArea currentArea = closestArea;
    CNavArea parentArea  = currentArea.Parent;

    while (parentArea != INVALID_NAV_AREA)
    {
        float flEdgeMiddle[3], flHalfWidth;
        
        float flTempCenter[3];
        currentArea.GetCenter(flTempCenter);
        int iDir = parentArea.ComputeDirection(flTempCenter);
        
        if (!parentArea.ComputePortal(currentArea, iDir, flEdgeMiddle, flHalfWidth))
            parentArea.GetCenter(flEdgeMiddle);
        
        flEdgeMiddle[2] = parentArea.GetZ(flEdgeMiddle);

        if ((parentArea.Attributes & NAV_MESH_PRECISE) && parentArea != startArea)
        {
            parentArea.GetCenter(flEdgeMiddle);
            flEdgeMiddle[2] = parentArea.GetZ(flEdgeMiddle);
        }

        g_aPathNodes[client].PushArray(flEdgeMiddle, 3);
        
        currentArea = parentArea;
        parentArea  = currentArea.Parent;
    }

    g_iCurrentNode[client] = g_aPathNodes[client].Length - 1;

    return bCompletePath;
}

void FollowPath(int client, float vel[3], float angles[3], int &buttons, float flNow)
{
    if (g_iCurrentNode[client] < 0 || g_aPathNodes[client].Length == 0 || g_iCurrentNode[client] >= g_aPathNodes[client].Length)
    {
        g_flStuckTime[client] = 0.0;
        vel[0] = 0.0;
        vel[1] = 0.0;
        vel[2] = 0.0;
        return;
    }

    float flBotPos[3];
    GetClientAbsOrigin(client, flBotPos);

    CNavArea botArea = GetAreaAtFeet(flBotPos);

    if (botArea != INVALID_NAV_AREA && (botArea.Attributes & NAV_MESH_CROUCH))
        buttons |= IN_DUCK;
    
    if (botArea != INVALID_NAV_AREA && (botArea.Attributes & NAV_MESH_JUMP))
    {
        int nOldButtonsJump = GetEntProp(client, Prop_Data, "m_nOldButtons");
        SetEntProp(client, Prop_Data, "m_nOldButtons", nOldButtonsJump & ~IN_JUMP);
        buttons |= IN_JUMP;
    }
    
    ///////////////////////////
    /* Basic stuck detection */
    ///////////////////////////
    
    float flCheckPos[3];
    flCheckPos    = flBotPos;
    flCheckPos[2] = g_flStuckCheckPos[client][2]; 

    float flCheckRef[3];
    flCheckRef = g_flStuckCheckPos[client];

    if (GetVectorDistance(flCheckPos, flCheckRef) > STUCK_RADIUS)
    {
        g_flStuckCheckPos[client] = flBotPos;
        g_flStuckTime[client]     = 0.0;
    }
    else
    {
        if (g_flStuckTime[client] == 0.0) g_flStuckTime[client] = flNow;
        float stuckDuration = flNow - g_flStuckTime[client];
        if (stuckDuration >= STUCK_WAIT_TIME)
        {
            if (GetEntityFlags(client) & FL_ONGROUND)
            {
                int nOldButtons = GetEntProp(client, Prop_Data, "m_nOldButtons");
                SetEntProp(client, Prop_Data, "m_nOldButtons", nOldButtons & ~(IN_JUMP | IN_DUCK));
                buttons |= IN_JUMP;
            }
            if (stuckDuration >= STUCK_WAIT_TIME + 0.5)
            {
                buttons |= IN_DUCK;
                g_flStuckTime[client]     = flNow;
                g_flStuckCheckPos[client] = flBotPos; 
            }
        }
    }

    /////////////////////
    /* Advancing nodes */
    /////////////////////
    
    float flNodePos[3];
    g_aPathNodes[client].GetArray(g_iCurrentNode[client], flNodePos);

    float vecDiff[3];
    SubtractVectors(flNodePos, flBotPos, vecDiff);
    vecDiff[2] = 0.0; 
    float flDist2D = GetVectorLength(vecDiff);
    
    if (flDist2D < 32.0)
    {
        g_iCurrentNode[client]--;
        return;
    }

    CNavArea nodeArea = GetAreaAtFeet(flNodePos);

    if (botArea != INVALID_NAV_AREA && nodeArea != INVALID_NAV_AREA && botArea == nodeArea)
    {
        g_iCurrentNode[client]--;
        return;
    }

    MoveTo(client, flNodePos, angles, vel);
}

public Action OnPlayerRunCmd(int client, int &buttons, int &impulse, float vel[3], float angles[3], int &weapon, int &subtype, int &cmdnum, int &tickcount, int &seed, int mouse[2])
{
    if (!IsFakeClient(client) || !IsPlayerAlive(client))
        return Plugin_Continue;
        
    if (!g_bRoundLive)
        return Plugin_Changed;

    float flNow  = GetGameTime();
    float flDist = 999999.0;

    float flBotPos[3];
    GetClientAbsOrigin(client, flBotPos);

    //////////////////////////////////////////////////
    /* Teleport to nearest human when totally stuck */
    //////////////////////////////////////////////////
    
    if (g_flLastMoveTime[client] == 0.0)
    {
        g_flLastMoveTime[client] = flNow;
        g_flLastMovePos[client]  = flBotPos;
    }
    else
    {
        float vecIdleDiff[3];
        SubtractVectors(flBotPos, g_flLastMovePos[client], vecIdleDiff);
        
        if (GetVectorLength(vecIdleDiff) > 25.0)
        {
            g_flLastMovePos[client]  = flBotPos;
            g_flLastMoveTime[client] = flNow;
        }
        else if (flNow - g_flLastMoveTime[client] >= g_cvMaxHardstuckTime.FloatValue)
        {
            int targetHuman = FindNearestZombiePlayer(client);
            if (targetHuman != -1)
            {
                float targetPos[3];
                GetClientAbsOrigin(targetHuman, targetPos);
                targetPos[2] += 15.0; 

                TeleportEntity(client, targetPos, NULL_VECTOR, NULL_VECTOR);

                g_flLastMovePos[client]  = targetPos;
                g_flLastMoveTime[client] = flNow;

                g_aPathNodes[client].Clear();
                g_iCurrentNode[client]   = -1;
                g_hLastEnemyArea[client] = INVALID_NAV_AREA;

                return Plugin_Changed;
            }
        }
    }
    
    int iEnemy = FindNearestEnemy(client);

    if (iEnemy != -1 && IsClientInGame(iEnemy) && IsPlayerAlive(iEnemy))
    {
        float flEnemyPos[3];
        GetClientAbsOrigin(iEnemy, flEnemyPos);
        flDist = GetVectorDistance(flBotPos, flEnemyPos);

        CNavArea botArea   = GetAreaAtFeet(flBotPos);
        CNavArea enemyArea = GetAreaAtFeet(flEnemyPos);
        bool bSameArea     = (botArea != INVALID_NAV_AREA && enemyArea != INVALID_NAV_AREA && botArea == enemyArea);

        float flEnemyEye[3];
        GetClientEyePosition(iEnemy, flEnemyEye);

        SmoothAimAt(client, flEnemyEye, angles, 14.0);
        TeleportEntity(client, NULL_VECTOR, angles, NULL_VECTOR);

        if (flDist <= AIM_UPDATE_RANGE)
            buttons |= IN_ATTACK;

        if (enemyArea != INVALID_NAV_AREA && enemyArea != g_hLastEnemyArea[client])
        {
            g_hLastEnemyArea[client] = enemyArea;
            if (g_cvRecalcOnNode != null && g_cvRecalcOnNode.BoolValue)
                g_flNextPathUpdate[client] = flNow;
        }

        if (flNow >= g_flNextPathUpdate[client])
        {
            BuildNavPath(client, flEnemyPos);
            g_flNextPathUpdate[client] = flNow + PATH_UPDATE_RATE;
        }

        if (bSameArea)
        {
            MoveTo(client, flEnemyPos, angles, vel);
            return Plugin_Changed;
        }
    }
    else
    {
        g_aPathNodes[client].Clear();
        g_iCurrentNode[client]   = -1;
        g_hLastEnemyArea[client] = INVALID_NAV_AREA;

        vel[0] = 0.0;
        vel[1] = 0.0;
        vel[2] = 0.0;
        return Plugin_Changed;
    }

    FollowPath(client, vel, angles, buttons, flNow);

    return Plugin_Changed;
}


/////////////
/* Helpers */
/////////////

CNavArea GetAreaAtFeet(const float flPos[3], int iTeam = -1)
{
    float flProjected[3];
    flProjected = flPos;

    float flGroundZ;
    float flNormal[3];
    if (NavMesh_GetGroundHeight(flProjected, flGroundZ, flNormal))
        flProjected[2] = flGroundZ;

    return NavMesh_GetNearestArea(flProjected, false, 150.0, false, false, iTeam);
}

int FindNearestEnemy(int client)
{
    int   iBest      = -1;
    float flBestDist = 999999.0;
    float flBotPos[3];
    GetClientAbsOrigin(client, flBotPos);

    for (int i = 1; i <= MaxClients; i++)
    {
        if (i == client)
	        continue;
        if (!IsClientInGame(i))
	        continue;
        if (!IsPlayerAlive(i))
	        continue;
        if (GetClientTeam(i) == GetClientTeam(client))
	        continue;
        if (TF2_IsPlayerInCondition(i, TFCond_Cloaked))
	        continue;
        
        float flPos[3];
        GetClientAbsOrigin(i, flPos);
        float flDist = GetVectorDistance(flBotPos, flPos);
        if (flDist < flBestDist) 
        	flBestDist = flDist; iBest = i;
    }
    return iBest;
}

int FindNearestZombiePlayer(int client)
{
    int   iBest      = -1;
    float flBestDist = 999999.0;
    float flBotPos[3];
    GetClientAbsOrigin(client, flBotPos);

    for (int i = 1; i <= MaxClients; i++)
    {
        if (i == client)
	        continue;
        if (!IsClientInGame(i))
	        continue;
        if (!IsPlayerAlive(i))
	        continue;
        if (IsFakeClient(i))
	        continue; 
        if (GetClientTeam(i) != 3)
	        continue; 

        float flPos[3];
        GetClientAbsOrigin(i, flPos);
        float flDist = GetVectorDistance(flBotPos, flPos);
        if (flDist < flBestDist) 
        	flBestDist = flDist; iBest = i;
    }
    return iBest;
}

void SmoothAimAt(int iClient, const float vTargetPos[3], float vAngleInOut[3], float flSmoothVal = 8.0)
{
    float vClientEyes[3], vIdealAng[3];
    GetClientEyePosition(iClient, vClientEyes);
    
    float vDirection[3];
    SubtractVectors(vTargetPos, vClientEyes, vDirection);

    if (GetVectorLength(vDirection) < 1.0) return;

    GetVectorAngles(vDirection, vIdealAng);
    
    if (vIdealAng[0] > 180.0) vIdealAng[0] -= 360.0;

    float flCurPitch = g_flDesiredAngles[iClient][0];
    float flCurYaw   = g_flDesiredAngles[iClient][1];

    float flDeltaPitch = vIdealAng[0] - flCurPitch;
    float flDeltaYaw   = vIdealAng[1] - flCurYaw;

    while (flDeltaYaw > 180.0)  flDeltaYaw -= 360.0;
    while (flDeltaYaw < -180.0) flDeltaYaw += 360.0;

    g_flAimMomentX[iClient] = (flDeltaPitch / flSmoothVal);
    g_flAimMomentY[iClient] = (flDeltaYaw / flSmoothVal);

    float flNewPitch = flCurPitch + g_flAimMomentX[iClient];
    float flNewYaw   = flCurYaw   + g_flAimMomentY[iClient];

    if (flNewPitch > 89.0)       flNewPitch = 89.0;
    else if (flNewPitch < -89.0) flNewPitch = -89.0;

    while (flNewYaw > 180.0)  flNewYaw -= 360.0;
    while (flNewYaw < -180.0) flNewYaw += 360.0;

    vAngleInOut[0] = flNewPitch;
    vAngleInOut[1] = flNewYaw;
    vAngleInOut[2] = 0.0;

    g_flDesiredAngles[iClient][0] = flNewPitch;
    g_flDesiredAngles[iClient][1] = flNewYaw;
    g_flDesiredAngles[iClient][2] = 0.0;
}

void MoveTo(int client, const float flGoal[3], const float flAng[3], float vel[3])
{
    float flPos[3], flWorldDir[3];
    GetClientAbsOrigin(client, flPos);
    
    SubtractVectors(flGoal, flPos, flWorldDir);

    flWorldDir[2] = 0.0;
    NormalizeVector(flWorldDir, flWorldDir);

    float flSin = Sine(DegToRad(flAng[1]));
    float flCos = Cosine(DegToRad(flAng[1]));

    vel[0] = flWorldDir[0] * flCos + flWorldDir[1] * flSin;
    vel[1] = flWorldDir[0] * flSin - flWorldDir[1] * flCos;
    vel[2] = 0.0;
    
    float flSpeed = GetEntPropFloat(client, Prop_Data, "m_flMaxspeed");
    NormalizeVector(vel, vel);
    ScaleVector(vel, (flSpeed > 0.0) ? flSpeed : 300.0);
}

public int TFPathCost(CNavArea area, CNavArea fromArea, CNavLadder ladder, any data)
{
    if (fromArea == INVALID_NAV_AREA) return 0;

    float flAreaCenter[3], flFromCenter[3];
    area.GetCenter(flAreaCenter);
    fromArea.GetCenter(flFromCenter);

    float cost = GetVectorDistance(flAreaCenter, flFromCenter);
    
    // Dont prefer climbing uphill
    if (fromArea.GetZ(flFromCenter) - area.GetZ(flAreaCenter) < -18.0)
        cost *= 2.0;

    if (area.Attributes & NAV_MESH_PRECISE)
        cost *= 0.25; // Give nav_precise preferential treatment
    
    return RoundToNearest(cost) + fromArea.CostSoFar;
}

////////////
/* Events */
////////////

public void Event_DoorOpened(const char[] output, int caller, int activator, float delay)
{
    int navInterface = FindEntityByClassname(-1, "tf_point_nav_interface");
    
    if (navInterface == -1)
    {
        navInterface = CreateEntityByName("tf_point_nav_interface");
        if (navInterface != -1)
            DispatchSpawn(navInterface);
    }
    
    if (navInterface != -1)
        AcceptEntityInput(navInterface, "RecomputeBlockers");
}

public void Event_RoundStart(Event event, const char[] name, bool dontBroadcast)
{
    g_bRoundLive = false;
    for (int i = 1; i <= MaxClients; i++)
    {
        if (!IsClientInGame(i) || !IsFakeClient(i)) continue;
        g_bDidInitialDeath[i] = false;
        if (GetClientTeam(i) != 3) ChangeClientTeam(i, 3);
    }
}

public void Event_RoundActive(Event event, const char[] name, bool dontBroadcast)
{
    g_bRoundLive = true;
}

public void Event_PlayerSpawn(Event event, const char[] name, bool dontBroadcast)
{
    int client = GetClientOfUserId(event.GetInt("userid"));
    if (client < 1 || !IsClientInGame(client) || !IsFakeClient(client)) return;

    if (!g_bDidInitialDeath[client])
    {
        g_bDidInitialDeath[client] = true;
        ForcePlayerSuicide(client);
        return;
    }

    g_aPathNodes[client].Clear();
    g_iCurrentNode[client]       = -1;
    g_iTargetEnemy[client]       = -1;
    g_flStuckTime[client]        = 0.0;
    g_hLastEnemyArea[client]     = INVALID_NAV_AREA;
    g_flAimMomentX[client]       = 0.0;
    g_flAimMomentY[client]       = 0.0;
    g_flDesiredAngles[client][0] = 0.0;
    g_flDesiredAngles[client][1] = 0.0;
    g_flDesiredAngles[client][2] = 0.0;
    g_flStuckCheckPos[client][0] = 0.0;
    g_flStuckCheckPos[client][1] = 0.0;
    g_flStuckCheckPos[client][2] = 0.0;
    g_flLastMoveTime[client]     = 0.0; 

    FakeClientCommand(client, "slot3");
    g_flNextPathUpdate[client] = GetGameTime() + GetRandomFloat(0.1, 1.5);
}
