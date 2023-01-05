    --------------------[ HAT LINKS ]--------------------
    
    --https://www.roblox.com/catalog/6904899146/Star-Man
    --https://www.roblox.com/catalog/11398414291/Doodled-Mask-Smug
    --https://www.roblox.com/catalog/12548563/Asimov-Outlaw
    --https://www.roblox.com/catalog/20011897/Space-Cop
    --https://www.roblox.com/catalog/6211939438/Loose-Black-Anime-Side-Buns
    --https://www.roblox.com/catalog/63690008/Pal-Hair
    --https://www.roblox.com/catalog/48474313/Red-Roblox-Cap
    --https://www.roblox.com/catalog/62724852/Chestnut-Bun
    --https://www.roblox.com/catalog/48474294/ROBLOX-Girl-Hair
    
    --Cost: 292
    
    --------------------[ END LIST ]--------------------
    
    

game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false


local controlslist = "Controls:                             q = bang                                   e = laying sex                                          r = blowjob                                      t = sidefuck                                     y = penis                                      z = light dash                                u = barrage                                click = attack                                l = mount                                x + click = shoot arm                                c = full nelson                                v =  mommy                                f =  carry                                b =  headsit"

--------------------[ HATS LMAO ]--------------------

local torsohat = "Star ManAccessory"
local headhat = "Meshes/mask2 (1)Accessory"
local rightarmhat = "Kate Hair"
local leftarmhat = "Robloxclassicred"
local rightleghat = "Pink Hair"
local leftleghat = "Pal Hair"
local rightarmhat = "Kate Hair"
local cheek1hat = "SpaceHelmetB"
local cheek2hat = "Space Cop"
local hairaccessory = "LooseSideBuns"


--------------------[ WHAT THE FUCK ]--------------------

-- SCRIPT MADE BY CGX_JX#1035

local clientavatar = false
local firstpersonTOG = false
local animspeed = 1 -- the range is 1 through 5 . dont change this unless u know what ur doing
local cam = game.Workspace.CurrentCamera
local CamPos,TargetCamPos = cam.CoordinateFrame.p,cam.CoordinateFrame.p 
local AngleX,TargetAngleX = 0,0
local AngleY,TargetAngleY = 0,0
local player = game.Players.LocalPlayer
local character = player.Character or player.CharacterAdded:wait()
local root = character:WaitForChild("HumanoidRootPart")


local humanoidpart = character.HumanoidRootPart
for i,v in pairs(game.Players.LocalPlayer.Character:GetChildren()) do
	if v.Name == rightarmhat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == leftleghat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == rightleghat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == leftarmhat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == cheek2hat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == cheek1hat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end

	if v.Name == torsohat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
	if v.Name == rightarmhat then
		for i,va in pairs(v:GetDescendants()) do
			if va:IsA("SpecialMesh") then
				va:Destroy()
			end
		end
	end
end
wait()
equipsounds = true    --gets sometimes annoying
scopeimage = true     --WILL MAYBE NOT WORK DUE TO MONITOR ISSUES  thats why its on a switch
necko=CFrame.new(0, 1, 0, -1, -0, -0, 0, 0, 1, 0, 1, 0)
necko2=CFrame.new(0, -0.5, 0, -1, -0, -0, 0, 0, 1, 0, 1, 0)
local thingysomething = true
local animationspeed = 0.6
function swait(num)
	if num == 0 or num == nil then
		game:service("RunService").Stepped:wait()
	else
		for i = 0, num do
			game:service("RunService").Stepped:wait()
		end
	end
end
local jetpack = false
local kicking = false
local armshooting = false
local animmode = ""
local crouching = false
local standrape = Instance.new('BoolValue')
standrape.Value = false
local freemouse = true
local sin = math.sin
local euler = CFrame.fromEulerAnglesXYZ

--reanimate by MyWorld#4430 discord.gg/pYVHtSJmEY
local Vector3_101 = Vector3.new(1, 0, 1)
local netless_Y = Vector3.new(0, 25.1, 0)
local function getNetlessVelocity(realPartVelocity)
	local mag = realPartVelocity.Magnitude
	if (mag > 1) and (mag < 100) then
		local unit = realPartVelocity.Unit
		if (unit.Y > 0.25) or (unit.Y < -0.75) then
			return realPartVelocity * (25.1 / realPartVelocity.Y)
		end
		realPartVelocity = unit * 125
	end
	return (realPartVelocity * Vector3_101) + netless_Y
end
local simradius = "shp" --simulation radius (net bypass) method
--"shp" - sethiddenproperty
--"ssr" - setsimulationradius
--false - disable
local simrad = 1000 --simulation radius value
local healthHide = true --moves your head away every 3 seconds so players dont see your health bar (alignmode 4 only)
local reclaim = true --if you lost control over a part this will move your primary part to the part so you get it back (alignmode 4)
local novoid = true --prevents parts from going under workspace.FallenPartsDestroyHeight if you control them (alignmode 4 only)
local physp = nil --PhysicalProperties.new(0.01, 0, 1, 0, 0) --sets .CustomPhysicalProperties to this for each part
local noclipAllParts = false --set it to true if you want noclip
local antiragdoll = true --removes hingeConstraints and ballSocketConstraints from your character
local newanimate = false --disables the animate script and enables after reanimation
local discharscripts = true --disables all localScripts parented to your character before reanimation
local R15toR6 = true --tries to convert your character to r6 if its r15
local hatcollide = true --makes hats cancollide (credit to ShownApe) (works only with reanimate method 0)
local humState16 = true --enables collisions for limbs before the humanoid dies (using hum:ChangeState)
local addtools = false --puts all tools from backpack to character and lets you hold them after reanimation
local hedafterneck = true --disable aligns for head and enable after neck or torso is removed
local loadtime = game:GetService("Players").RespawnTime + 0.5 --anti respawn delay
local method = 3 --reanimation method
local sliding = false
local dancing = false
local running = true
--methods:
--0 - breakJoints (takes [loadtime] seconds to laod)
--1 - limbs
--2 - limbs + anti respawn
--3 - limbs + breakJoints after [loadtime] seconds
--4 - remove humanoid + breakJoints
--5 - remove humanoid + limbs
local alignmode = 4 --AlignPosition mode
--modes:
--1 - AlignPosition rigidity enabled true
--2 - 2 AlignPositions rigidity enabled both true and false
--3 - AlignPosition rigidity enabled false
--4 - CFrame (if u dont have the isnetworkowner function it will use alignmode 2)
local flingpart = nil--name of the part or the hat used for flinging
if game.Players.LocalPlayer.Character.Humanoid.RigType == Enum.HumanoidRigType.R6 then
	flingpart = "HumanoidRootPart"
end
if game.Players.LocalPlayer.Character.Humanoid.RigType == Enum.HumanoidRigType.R15 then
	flingpart = "HumanoidRootPart"
end --name of the part or the hat used for flinging
--the fling function
--usage: fling(target, duration, velocity)
--target can be set to: basePart, CFrame, Vector3, character model or humanoid (flings at mouse.Hit if argument not provided))
--duration (fling time in seconds) can be set to: a number or a string convertable to the number (0.5s if not provided),
--velocity (fling part rotation velocity) can be set to a vector3 value (Vector3.new(20000, 20000, 20000) if not provided)
local lp = game:GetService("Players").LocalPlayer
local mouse = lp:GetMouse()
local rs, ws, sg = game:GetService("RunService"), game:GetService("Workspace"), game:GetService("StarterGui")
local stepped, heartbeat, renderstepped = rs.Stepped, rs.Heartbeat, rs.RenderStepped
local twait, tdelay, rad, inf, abs, clamp = task.wait, task.delay, math.rad, math.huge, math.abs, math.clamp
local cf, v3 = CFrame.new, Vector3.new
local angles = CFrame.Angles
local v3_0, cf_0 = v3(0, 0, 0), cf(0, 0, 0)

local Humanoid = lp.Character:FindFirstChildOfClass("Humanoid")
local raycast = ws.Raycast
local HumanoidRootPart = lp.Character.HumanoidRootPart


local inf = math.huge

local c = lp.Character

if not (c and c.Parent) then
	return
end

c:GetPropertyChangedSignal("Parent"):Connect(function()
	if not (c and c.Parent) then
		c = nil
	end
end)

local function gp(parent, name, className)
	if typeof(parent) == "Instance" then
		for i, v in pairs(parent:GetChildren()) do
			if (v.Name == name) and v:IsA(className) then
				return v
			end
		end
	end
	return nil
end

if type(getNetlessVelocity) ~= "function" then
	getNetlessVelocity = nil
end

local fenv = getfenv()
local shp = fenv.sethiddenproperty or fenv.set_hidden_property or fenv.set_hidden_prop or fenv.sethiddenprop
local ssr = fenv.setsimulationradius or fenv.set_simulation_radius or fenv.set_sim_radius or fenv.setsimradius or fenv.set_simulation_rad or fenv.setsimulationrad
local ino = fenv.isnetworkowner or fenv.is_network_owner or fenv.isnetowner or fenv.is_net_owner

if (alignmode == 4) and (not ino) then
	alignmode = 2
end

local physp = PhysicalProperties.new(0.01, 0, 1, 0, 0)

local swingvalue = 1
local function align(Part0, Part1)

	local att0 = Instance.new("Attachment")
	att0.Position, att0.Orientation, att0.Name = v3_0, v3_0, "att0_" .. Part0.Name
	local att1 = Instance.new("Attachment")
	att1.Position, att1.Orientation, att1.Name = v3_0, v3_0, "att1_" .. Part1.Name

	if alignmode == 4 then

		local hide = false
		if Part0 == healthHide then
			healthHide = false
			tdelay(0, function()
				while twait(2.9) and Part0 and c do
					hide = #Part0:GetConnectedParts() == 1
					twait(0.1)
					hide = false
				end
			end)
		end

		local rot = rad(0.05)
		local con0, con1 = nil, nil

		con0 = stepped:Connect(function()
			if not (Part0 and Part1) then return con0:Disconnect() and con1:Disconnect() end
			Part0.RotVelocity = Part1.RotVelocity
		end)
		local lastpos = Part0.Position
		con1 = heartbeat:Connect(function(delta)
			if not (Part0 and Part1 and att1) then return con0:Disconnect() and con1:Disconnect() end
			if (not Part0.Anchored) and (Part0.ReceiveAge == 0) then
				if lostpart == Part0 then
					lostpart = nil
				end
				rot = -rot
				local newcf = Part1.CFrame * att1.CFrame * angles(0, 0, rot)
				if Part1.Velocity.Magnitude > 0.01 then
					Part0.Velocity = getNetlessVelocity(Part1.Velocity)
				else
					Part0.Velocity = getNetlessVelocity((newcf.Position - lastpos) / delta)
				end
				lastpos = newcf.Position
				if lostpart and (Part0 == reclaim) then
					newcf = lostpart.CFrame
				elseif hide then
					newcf += v3(0, 3000, 0)
				end
				if novoid and (newcf.Y < ws.FallenPartsDestroyHeight + 0.1) then
					newcf += v3(0, ws.FallenPartsDestroyHeight + 0.1 - newcf.Y, 0)
				end
				Part0.CFrame = newcf
			elseif (not Part0.Anchored) and (abs(Part0.Velocity.X) < 45) and (abs(Part0.Velocity.Y) < 25) and (abs(Part0.Velocity.Z) < 45) then
				lostpart = Part0
			end
		end)

	else

		Part0.CustomPhysicalProperties = physp
		if (alignmode == 1) or (alignmode == 2) then
			local ape = Instance.new("AlignPosition")
			ape.MaxForce, ape.MaxVelocity, ape.Responsiveness = inf, inf, inf
			ape.ReactionForceEnabled, ape.RigidityEnabled, ape.ApplyAtCenterOfMass = false, true, false
			ape.Attachment0, ape.Attachment1, ape.Name = att0, att1, "AlignPositionRtrue"
			ape.Parent = att0
		end

		if (alignmode == 2) or (alignmode == 3) then
			local apd = Instance.new("AlignPosition")
			apd.MaxForce, apd.MaxVelocity, apd.Responsiveness = inf, inf, inf
			apd.ReactionForceEnabled, apd.RigidityEnabled, apd.ApplyAtCenterOfMass = false, false, false
			apd.Attachment0, apd.Attachment1, apd.Name = att0, att1, "AlignPositionRfalse"
			apd.Parent = att0
		end

		local ao = Instance.new("AlignOrientation")
		ao.MaxAngularVelocity, ao.MaxTorque, ao.Responsiveness = inf, inf, inf
		ao.PrimaryAxisOnly, ao.ReactionTorqueEnabled, ao.RigidityEnabled = false, false, false
		ao.Attachment0, ao.Attachment1 = att0, att1
		ao.Parent = att0

		local con0, con1 = nil, nil
		local vel = Part0.Velocity
		con0 = renderstepped:Connect(function()
			if not (Part0 and Part1) then return con0:Disconnect() and con1:Disconnect() end
			Part0.Velocity = vel
		end)
		local lastpos = Part0.Position
		con1 = heartbeat:Connect(function(delta)
			if not (Part0 and Part1) then return con0:Disconnect() and con1:Disconnect() end
			vel = Part0.Velocity
			if Part1.Velocity.Magnitude > 0.01 then
				Part0.Velocity = getNetlessVelocity(Part1.Velocity)
			else
				Part0.Velocity = getNetlessVelocity((Part0.Position - lastpos) / delta)
			end
			lastpos = Part0.Position
		end)

	end

	att0:GetPropertyChangedSignal("Parent"):Connect(function()
		Part0 = att0.Parent
		if not Part0:IsA("BasePart") then
			att0 = nil
			if lostpart == Part0 then
				lostpart = nil
			end
			Part0 = nil
		end
	end)
	att0.Parent = Part0

	att1:GetPropertyChangedSignal("Parent"):Connect(function()
		Part1 = att1.Parent
		if not Part1:IsA("BasePart") then
			att1 = nil
			Part1 = nil
		end
	end)
	att1.Parent = Part1
end

local function respawnrequest()
	cam.CameraSubject = game.Players.LocalPlayer.Character:FindFirstChildOfClass("Humanoid")
	cam.CameraType = Enum.CameraType.Follow
	--cam.CFrame = game.Players.LocalPlayer.Character:FindFirstChildOfClass("Head").CFrame
	local ccfr = ws.CurrentCamera.CFrame
	local c = lp.Character
	lp.Character = nil
	lp.Character = c
	local con = nil
	con = ws.CurrentCamera.Changed:Connect(function(prop)
		if (prop ~= "Parent") and (prop ~= "CFrame") then
			return
		end
		ws.CurrentCamera.CFrame = ccfr
		con:Disconnect()
	end)
end

local destroyhum = (method == 4) or (method == 5)
local breakjoints = (method == 0) or (method == 4)
local antirespawn = (method == 0) or (method == 2) or (method == 3)

hatcollide = hatcollide and (method == 0)

addtools = addtools and gp(lp, "Backpack", "Backpack")

if shp and (simradius == "shp") then
	spawn(function()
		while c and heartbeat:Wait() do
			shp(lp, "SimulationRadius", inf)
		end
	end)
elseif ssr and (simradius == "ssr") then
	spawn(function()
		while c and heartbeat:Wait() do
			ssr(inf)
		end
	end)
end

antiragdoll = antiragdoll and function(v)
	if v:IsA("HingeConstraint") or v:IsA("BallSocketConstraint") then
		v.Parent = nil
	end
end

if antiragdoll then
	for i, v in pairs(c:GetDescendants()) do
		antiragdoll(v)
	end
	c.DescendantAdded:Connect(antiragdoll)
end

if antirespawn then
	respawnrequest()
end

if method == 0 then
	wait(loadtime)
	if not c then
		return
	end
end

if discharscripts then
	for i, v in pairs(c:GetChildren()) do
		if v:IsA("LocalScript") then
			v.Enabled = false
		end
	end
elseif newanimate then
	local animate = gp(c, "Animate", "LocalScript")
	if animate and (not animate.Enabled) then
		animate.Enabled = false
	else
		newanimate = false
	end
end

if addtools then
	for i, v in pairs(addtools:GetChildren()) do
		if v:IsA("Tool") then
			v.Parent = c
		end
	end
end

pcall(function()
	settings().Physics.AllowSleep = false
	settings().Physics.PhysicsEnvironmentalThrottle = Enum.EnviromentalPhysicsThrottle.Disabled
end)

local OLDscripts = {}

for i, v in pairs(c:GetDescendants()) do
	if v.ClassName == "Script" then
		table.insert(OLDscripts, v)
	end
end

local scriptNames = {}

for i, v in pairs(c:GetDescendants()) do
	if v:IsA("BasePart") then
		local newName = tostring(i)
		local exists = true
		while exists do
			exists = false
			for i, v in pairs(OLDscripts) do
				if v.Name == newName then
					exists = true
				end
			end
			if exists then
				newName = newName .. "_"    
			end
		end
		table.insert(scriptNames, newName)
		Instance.new("Script", v).Name = newName
	end
end

c.Archivable = true
local hum = c:FindFirstChildOfClass("Humanoid")
if hum then
	for i, v in pairs(hum:GetPlayingAnimationTracks()) do
		v:Stop()
	end
end
local cl = c:Clone()
if hum and humState16 then
	hum:ChangeState(Enum.HumanoidStateType.Physics)
	if destroyhum then
		wait(1.6)
	end
end
if hum and hum.Parent and destroyhum then
	hum:Destroy()
end

if not c then
	return
end

local head = gp(c, "Head", "BasePart")
local torso = gp(c, "Torso", "BasePart") or gp(c, "UpperTorso", "BasePart")
local root = gp(c, "HumanoidRootPart", "BasePart")
if hatcollide and c:FindFirstChildOfClass("Accessory") then
	local anything = c:FindFirstChildOfClass("BodyColors") or gp(c, "Health", "Script")
	if not (torso and root and anything) then
		return
	end
	torso:Destroy()
	root:Destroy()
	anything:Destroy()
end

local model = Instance.new("Model", c)
model:GetPropertyChangedSignal("Parent"):Connect(function()
	if not (model and model.Parent) then
		model = nil
	end
end)

for i, v in pairs(c:GetChildren()) do
	if v ~= model then
		if addtools and v:IsA("Tool") then
			for i1, v1 in pairs(v:GetDescendants()) do
				if v1 and v1.Parent and v1:IsA("BasePart") then
					local bv = Instance.new("BodyVelocity", v1)
					bv.Velocity = v3_0
					bv.MaxForce = v3(1000, 1000, 1000)
					bv.P = 1250
					bv.Name = "bv_" .. v.Name

				end
			end
		end
		v.Parent = model
	end
end

if breakjoints then
	model:BreakJoints()
else
	if head and torso then
		for i, v in pairs(model:GetDescendants()) do
			if v:IsA("JointInstance") then
				local save = false
				if (v.Part0 == torso) and (v.Part1 == head) then
					save = true
				end
				if (v.Part0 == head) and (v.Part1 == torso) then
					save = true
				end
				if save then
					if hedafterneck then
						hedafterneck = v
					end
				else
					v:Destroy()
				end
			end
		end
	end
	if method == 3 then
		task.delay(loadtime, pcall, model.BreakJoints, model)
	end
end
local lerp = function(a, b, t)
	return a * (1 - t) + b * t
end

function clerp(a,b,t)
	local qa = {QuaternionFromCFrame(a)}
	local qb = {QuaternionFromCFrame(b)}
	local ax, ay, az = a.x, a.y, a.z
	local bx, by, bz = b.x, b.y, b.z
	local _t = 1-t
	return QuaternionToCFrame(_t*ax + t*bx, _t*ay + t*by, _t*az + t*bz,QuaternionSlerp(qa, qb, t))
end

function QuaternionFromCFrame(cf)
	local mx, my, mz, m00, m01, m02, m10, m11, m12, m20, m21, m22 = cf:components()
	local trace = m00 + m11 + m22
	if trace > 0 then
		local s = math.sqrt(1 + trace)
		local recip = 0.5/s
		return (m21-m12)*recip, (m02-m20)*recip, (m10-m01)*recip, s*0.5
	else
		local i = 0
		if m11 > m00 then
			i = 1
		end
		if m22 > (i == 0 and m00 or m11) then
			i = 2
		end
		if i == 0 then
			local s = math.sqrt(m00-m11-m22+1)
			local recip = 0.5/s
			return 0.5*s, (m10+m01)*recip, (m20+m02)*recip, (m21-m12)*recip
		elseif i == 1 then
			local s = math.sqrt(m11-m22-m00+1)
			local recip = 0.5/s
			return (m01+m10)*recip, 0.5*s, (m21+m12)*recip, (m02-m20)*recip
		elseif i == 2 then
			local s = math.sqrt(m22-m00-m11+1)
			local recip = 0.5/s return (m02+m20)*recip, (m12+m21)*recip, 0.5*s, (m10-m01)*recip
		end
	end
end


function QuaternionToCFrame(px, py, pz, x, y, z, w)
	local xs, ys, zs = x + x, y + y, z + z
	local wx, wy, wz = w*xs, w*ys, w*zs
	local xx = x*xs
	local xy = x*ys
	local xz = x*zs
	local yy = y*ys
	local yz = y*zs
	local zz = z*zs
	return CFrame.new(px, py, pz,1-(yy+zz), xy - wz, xz + wy,xy + wz, 1-(xx+zz), yz - wx, xz - wy, yz + wx, 1-(xx+yy))
end
function QuaternionSlerp(a, b, t)
	local cosTheta = a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4]
	local startInterp, finishInterp;
	if cosTheta >= 0.0001 then
		if (1 - cosTheta) > 0.0001 then
			local theta = math.acos(cosTheta)
			local invSinTheta = 1/math.sin(theta)
			startInterp = math.sin((1-t)*theta)*invSinTheta
			finishInterp = math.sin(t*theta)*invSinTheta  
		else
			startInterp = 1-t
			finishInterp = t
		end
	else
		if (1+cosTheta) > 0.0001 then
			local theta = math.acos(-cosTheta)
			local invSinTheta = 1/math.sin(theta)
			startInterp = math.sin((t-1)*theta)*invSinTheta
			finishInterp = math.sin(t*theta)*invSinTheta
		else
			startInterp = t-1
			finishInterp = t
		end
	end
	return a[1]*startInterp + b[1]*finishInterp, a[2]*startInterp + b[2]*finishInterp, a[3]*startInterp + b[3]*finishInterp, a[4]*startInterp + b[4]*finishInterp
end
for i, v in pairs(cl:GetChildren()) do
	v.Parent = c
end
cl:Destroy()

local uncollide, noclipcon = nil, nil
if noclipAllParts then
	uncollide = function()
		if c then
			for i, v in pairs(c:GetDescendants()) do
				if v:IsA("BasePart") then
					v.CanCollide = false
				end
			end
		else
			noclipcon:Disconnect()
		end
	end
else
	uncollide = function()
		if model then
			for i, v in pairs(model:GetDescendants()) do
				if v:IsA("BasePart") then
					v.CanCollide = false
				end
			end
		else
			noclipcon:Disconnect()
		end
	end
end
noclipcon = stepped:Connect(uncollide)
uncollide()

for i, scr in pairs(model:GetDescendants()) do
	if (scr.ClassName == "Script") and table.find(scriptNames, scr.Name) then
		local Part0 = scr.Parent
		if Part0:IsA("BasePart") then
			for i1, scr1 in pairs(c:GetDescendants()) do
				if (scr1.ClassName == "Script") and (scr1.Name == scr.Name) and (not scr1:IsDescendantOf(model)) then
					local Part1 = scr1.Parent
					if (Part1.ClassName == Part0.ClassName) and (Part1.Name == Part0.Name) then

						align(Part0, Part1)
						scr:Destroy()
						scr1:Destroy()
						break
					end
				end
			end
		end
	end
end

for i, v in pairs(c:GetDescendants()) do
	if v and v.Parent and (not v:IsDescendantOf(model)) then
		if v:IsA("Decal") then
			v.Transparency = 1
		elseif v:IsA("BasePart") then
			v.Transparency = 1
			v.Anchored = false
		elseif v:IsA("ForceField") then
			v.Visible = false
		elseif v:IsA("Sound") then
			v.Playing = false
		elseif v:IsA("BillboardGui") or v:IsA("SurfaceGui") or v:IsA("ParticleEmitter") or v:IsA("Fire") or v:IsA("Smoke") or v:IsA("Sparkles") then
			v.Enabled = false
		end
	end
end

if newanimate then
	local animate = gp(c, "Animate", "LocalScript")
	if animate then
		animate.Disabled = false
	end
end

if addtools then
	for i, v in pairs(c:GetChildren()) do
		if v:IsA("Tool") then
			v.Parent = addtools
		end
	end
end

local hum0, hum1 = model:FindFirstChildOfClass("Humanoid"), c:FindFirstChildOfClass("Humanoid")
if hum0 then
	hum0:GetPropertyChangedSignal("Parent"):Connect(function()
		if not (hum0 and hum0.Parent) then
			hum0 = nil
		end
	end)
end
if hum1 then
	hum1:GetPropertyChangedSignal("Parent"):Connect(function()
		if not (hum1 and hum1.Parent) then
			hum1 = nil
		end
	end)

	ws.CurrentCamera.CameraSubject = hum1
	local camSubCon = nil
	local function camSubFunc()
		camSubCon:Disconnect()
		if c and hum1 then
			ws.CurrentCamera.CameraSubject = hum1
		end
	end
	camSubCon = renderstepped:Connect(camSubFunc)
	if hum0 then
		hum0:GetPropertyChangedSignal("Jump"):Connect(function()
			if hum1 then
				hum1.Jump = hum0.Jump
			end
		end)
	else
		respawnrequest()
	end
end

local rb = Instance.new("BindableEvent", c)
rb.Event:Connect(function()
	rb:Destroy()
	sg:SetCore("ResetButtonCallback", true)
	if destroyhum then
		if c then c:BreakJoints() end
		return
	end
	if model and hum0 and (hum0.Health > 0) then
		model:BreakJoints()
		hum0.Health = 0
	end
	if antirespawn then
		respawnrequest()
	end
end)
sg:SetCore("ResetButtonCallback", rb)

spawn(function()
	while wait() and c do
		if hum0 and hum1 then
			hum1.Jump = hum0.Jump
		end
	end
	sg:SetCore("ResetButtonCallback", true)
end)

R15toR6 = R15toR6 and hum1 and (hum1.RigType == Enum.HumanoidRigType.R15)
if R15toR6 then
	local part = gp(c, "HumanoidRootPart", "BasePart") or gp(c, "UpperTorso", "BasePart") or gp(c, "LowerTorso", "BasePart") or gp(c, "Head", "BasePart") or c:FindFirstChildWhichIsA("BasePart")
	if part then
		local cfr = part.CFrame
		local R6parts = { 
			head = {
				Name = "Head",
				Size = v3(2, 1, 1),
				R15 = {
					Head = 0
				}
			},
			torso = {
				Name = "Torso",
				Size = v3(2, 2, 1),
				R15 = {
					UpperTorso = 0.2,
					LowerTorso = -0.8
				}
			},
			root = {
				Name = "HumanoidRootPart",
				Size = v3(2, 2, 1),
				R15 = {
					HumanoidRootPart = 0
				}
			},
			leftArm = {
				Name = "Left Arm",
				Size = v3(1, 2, 1),
				R15 = {
					LeftHand = -0.849,
					LeftLowerArm = -0.174,
					LeftUpperArm = 0.415
				}
			},
			rightArm = {
				Name = "Right Arm",
				Size = v3(1, 2, 1),
				R15 = {
					RightHand = -0.849,
					RightLowerArm = -0.174,
					RightUpperArm = 0.415
				}
			},
			leftLeg = {
				Name = "Left Leg",
				Size = v3(1, 2, 1),
				R15 = {
					LeftFoot = -0.85,
					LeftLowerLeg = -0.29,
					LeftUpperLeg = 0.49
				}
			},
			rightLeg = {
				Name = "Right Leg",
				Size = v3(1, 2, 1),
				R15 = {
					RightFoot = -0.85,
					RightLowerLeg = -0.29,
					RightUpperLeg = 0.49
				}
			}
		}
		for i, v in pairs(c:GetChildren()) do
			if v:IsA("BasePart") then
				for i1, v1 in pairs(v:GetChildren()) do
					if v1:IsA("Motor6D") then
						v1.Part0 = nil
					end
				end
			end
		end
		part.Archivable = true
		for i, v in pairs(R6parts) do
			local part = part:Clone()
			part:ClearAllChildren()
			part.Name = v.Name
			part.Size = v.Size
			part.CFrame = cfr
			part.Anchored = false
			part.Transparency = 1
			part.CanCollide = false
			for i1, v1 in pairs(v.R15) do
				local R15part = gp(c, i1, "BasePart")
				local att = gp(R15part, "att1_" .. i1, "Attachment")
				if R15part then
					local weld = Instance.new("Weld", R15part)
					weld.Name = "Weld_" .. i1
					weld.Part0 = part
					weld.Part1 = R15part
					weld.C0 = cf(0, v1, 0)
					weld.C1 = cf(0, 0, 0)
					R15part.Massless = true
					R15part.Name = "R15_" .. i1
					R15part.Parent = part
					if att then
						att.Parent = part
						att.Position = v3(0, v1, 0)
					end
				end
			end
			part.Parent = c
			R6parts[i] = part
		end
		local R6joints = {
			neck = {
				Parent = R6parts.torso,
				Name = "Neck",
				Part0 = R6parts.torso,
				Part1 = R6parts.head,
				C0 = cf(0, 1, 0, -1, 0, 0, 0, 0, 1, 0, 1, -0),
				C1 = cf(0, -0.5, 0, -1, 0, 0, 0, 0, 1, 0, 1, -0)
			},
			torso = {
				Parent = R6parts.root,
				Name = "torso" ,
				Part0 = R6parts.root,
				Part1 = R6parts.torso,
				C0 = cf(0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 1, -0),
				C1 = cf(0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 1, -0)
			},
			rs = {
				Parent = R6parts.torso,
				Name = "Right Shoulder",
				Part0 = R6parts.torso,
				Part1 = R6parts.rightArm,
				C0 = cf(1, 0.5, 0, 0, 0, 1, 0, 1, -0, -1, 0, 0),
				C1 = cf(-0.5, 0.5, 0, 0, 0, 1, 0, 1, -0, -1, 0, 0)
			},
			ls = {
				Parent = R6parts.torso,
				Name = "Left Shoulder",
				Part0 = R6parts.torso,
				Part1 = R6parts.leftArm,
				C0 = cf(-1, 0.5, 0, 0, 0, -1, 0, 1, 0, 1, 0, 0),
				C1 = cf(0.5, 0.5, 0, 0, 0, -1, 0, 1, 0, 1, 0, 0)
			},
			rh = {
				Parent = R6parts.torso,
				Name = "Right Hip",
				Part0 = R6parts.torso,
				Part1 = R6parts.rightLeg,
				C0 = cf(1, -1, 0, 0, 0, 1, 0, 1, -0, -1, 0, 0),
				C1 = cf(0.5, 1, 0, 0, 0, 1, 0, 1, -0, -1, 0, 0)
			},
			lh = {
				Parent = R6parts.torso,
				Name = "Left Hip" ,
				Part0 = R6parts.torso,
				Part1 = R6parts.leftLeg,
				C0 = cf(-1, -1, 0, 0, 0, -1, 0, 1, 0, 1, 0, 0),
				C1 = cf(-0.5, 1, 0, 0, 0, -1, 0, 1, 0, 1, 0, 0)
			}
		}
		for i, v in pairs(R6joints) do
			local joint = Instance.new("Motor6D")
			for prop, val in pairs(v) do
				joint[prop] = val
			end
			R6joints[i] = joint
		end
		if hum1 then
			hum1.RigType = Enum.HumanoidRigType.R6
			hum1.HipHeight = 0
		end
	end
end

local torso1 = torso
torso = gp(c, "Torso", "BasePart") or ((not R15toR6) and gp(c, torso.Name, "BasePart"))
if (typeof(hedafterneck) == "Instance") and head and torso and torso1 then
	local conNeck = nil
	local conTorso = nil
	local contorso1 = nil
	local aligns = {}
	local function enableAligns()
		conNeck:Disconnect()
		conTorso:Disconnect()
		conTorso1:Disconnect()
		for i, v in pairs(aligns) do
			v.Enabled = true
		end
	end
	conNeck = hedafterneck.Changed:Connect(function(prop)
		if table.find({"Part0", "Part1", "Parent"}, prop) then
			enableAligns()
		end
	end)
	conTorso = torso:GetPropertyChangedSignal("Parent"):Connect(enableAligns)
	conTorso1 = torso1:GetPropertyChangedSignal("Parent"):Connect(enableAligns)
	for i, v in pairs(head:GetDescendants()) do
		if v:IsA("AlignPosition") or v:IsA("AlignOrientation") then
			i = tostring(i)
			aligns[i] = v
			v:GetPropertyChangedSignal("Parent"):Connect(function()
				aligns[i] = nil
			end)
			v.Enabled = false
		end
	end
end
local mposx,mposy,mposz = 0,0,0

mouse.Button1Down:connect(function()
	print(mposx)

	if mposx < 1200 and mposy < 1200 and mposz < 1200 and mposx > -1200 and mposy > -1200 and mposz > -1200 then
		if armshooting == true then
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -2.25)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.0125,false)
			animmode = "gunshot"
			standrightarm.Part0 = game.Workspace.Terrain
			tweenobject2(standrightarm,{C0 = CFrame.new(mouse.Hit.p)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			wait(1)

			standrightarm.Part0 = ch[torsohat].Handle
			animmode = ""	

		end
	end

end)
cam.CameraSubject = game.Players.LocalPlayer.Character:FindFirstChildOfClass('Humanoid')

local flingpart0 = gp(model, flingpart, "BasePart") or gp(gp(model, flingpart, "Accessory"), "Handle", "BasePart")
local flingpart1 = gp(c, flingpart, "BasePart") or gp(gp(c, flingpart, "Accessory"), "Handle", "BasePart")

local fling = function() end
if flingpart0 and flingpart1 then
	flingpart0:GetPropertyChangedSignal("Parent"):Connect(function()
		if not (flingpart0 and flingpart0.Parent) then
			flingpart0 = nil
			fling = function() end
		end
	end)
	flingpart0.Archivable = true
	flingpart1:GetPropertyChangedSignal("Parent"):Connect(function()
		if not (flingpart1 and flingpart1.Parent) then
			flingpart1 = nil
			fling = function() end
		end
	end)
	local att0 = gp(flingpart0, "att0_" .. flingpart0.Name, "Attachment")
	local att1 = gp(flingpart1, "att1_" .. flingpart1.Name, "Attachment")
	if att0 and att1 then
		att0:GetPropertyChangedSignal("Parent"):Connect(function()
			if not (att0 and att0.Parent) then
				att0 = nil
				fling = function() end
			end
		end)
		att1:GetPropertyChangedSignal("Parent"):Connect(function()
			if not (att1 and att1.Parent) then
				att1 = nil
				fling = function() end
			end
		end)
		local lastfling = nil
		local mouse = lp:GetMouse()
		fling = function(target, duration, rotVelocity)
			if typeof(target) == "Instance" then
				if target:IsA("BasePart") then
					target = target.Position
				elseif target:IsA("Model") then
					target = gp(target, "HumanoidRootPart", "BasePart") or gp(target, "Torso", "BasePart") or gp(target, "UpperTorso", "BasePart") or target:FindFirstChildWhichIsA("BasePart")
					if target then
						target = target.Position
					else
						return
					end
				elseif target:IsA("Humanoid") then
					target = target.Parent
					if not (target and target:IsA("Model")) then
						return
					end
					target = gp(target, "HumanoidRootPart", "BasePart") or gp(target, "Torso", "BasePart") or gp(target, "UpperTorso", "BasePart") or target:FindFirstChildWhichIsA("BasePart")
					if target then
						target = target.Position
					else
						return
					end
				else
					return
				end
			elseif typeof(target) == "CFrame" then
				target = target.Position
			elseif typeof(target) ~= "Vector3" then
				target = mouse.Hit
				if target then
					target = target.Position
				else
					return
				end
			end
			if target.Y < ws.FallenPartsDestroyHeight + 5 then
				target = v3(target.X, ws.FallenPartsDestroyHeight + 5, target.Z)
			end
			lastfling = target
			if type(duration) ~= "number" then
				duration = tonumber(duration) or 0.5
			end
			if typeof(rotVelocity) ~= "Vector3" then
				rotVelocity = v3(20000, 20000, 20000)
			end
			if not (target and flingpart0 and flingpart1 and att0 and att1) then
				return
			end
			local flingpart = flingpart0:Clone()
			flingpart.Transparency = 1
			flingpart.CanCollide = false
			flingpart.Name = "flingpart_" .. flingpart0.Name
			flingpart.Anchored = true
			flingpart.Velocity = v3_0
			flingpart.RotVelocity = v3_0
			flingpart.Position = target
			flingpart:GetPropertyChangedSignal("Parent"):Connect(function()
				if not (flingpart and flingpart.Parent) then
					flingpart = nil
				end
			end)
			flingpart.Parent = flingpart1
			if flingpart0.Transparency > 0.5 then
				flingpart0.Transparency = 0.4
			end
			att1.Parent = flingpart
			local con = nil
			local rotchg = v3(0, rotVelocity.Unit.Y * -1000, 0)
			con = heartbeat:Connect(function(delta)
				if target and (lastfling == target) and flingpart and flingpart0 and flingpart1 and att0 and att1 then
					flingpart.Orientation += rotchg * delta
					flingpart0.RotVelocity = rotVelocity
				else
					con:Disconnect()
				end
			end)
			if alignmode ~= 4 then
				local con = nil
				con = renderstepped:Connect(function()
					if flingpart0 and target then
						flingpart0.RotVelocity = v3_0
					else
						con:Disconnect()
					end
				end)
			end
			wait(duration)
			if lastfling ~= target then
				if flingpart then
					if att1 and (att1.Parent == flingpart) then
						att1.Parent = flingpart1
					end
					flingpart:Destroy()
				end
				return
			end
			target = nil
			if not (flingpart and flingpart0 and flingpart1 and att0 and att1) then
				return
			end
			flingpart0.RotVelocity = v3_0
			att1.Parent = flingpart1
			if flingpart then
				flingpart:Destroy()
			end
		end
	end
end
wait(0.1)



--start script--

tweenservice = game:GetService("TweenService")
p = game.Players.LocalPlayer
ch = p.Character
hum = ch:FindFirstChildOfClass("Humanoid")
root = ch.HumanoidRootPart
Root = ch.HumanoidRootPart
sine = 0
change = 1
cam = game.Workspace.CurrentCamera
cam.FieldOfView = 90


Hwait = true
RunS = game:GetService("RunService")
SD = RunS.Stepped
HB = RunS.Heartbeat

local mposx,mposy,mposz = 0,0,0

head = ch.Head
leftarm = ch['Left Arm']
rightarm = ch['Right Arm']
leftleg = ch['Left Leg']
rightleg = ch['Right Leg']
tor = ch['Torso']
for i,v in pairs(ch.Model:GetChildren()) do
	if v:IsA('BasePart') then
		if v.Name == "HumanoidRootPart" then
			v.Transparency = 0.95
		else
			v.Transparency = 0	
		end

	end
end
--	ch.Model['Left Leg'].Transparency = 0
--ch.Model['Right Leg'].Transparency = 0
--	ch.Model['Head'].Transparency = 0
--ch.Model['Torso'].Transparency = 0

print"test1"
istyping = false
local UserInputService = game:GetService('UserInputService')
UserInputService.TextBoxFocused:Connect(function(textbox)
	if textbox.Name == "ChatBar" then
		istyping = true
	end
end)


UserInputService.TextBoxFocusReleased:Connect(function(textbox)
	if textbox.Name == "ChatBar" then
		istyping = false
	end
end)


repeat wait() until game:GetService("Players").LocalPlayer.Character ~= nil
local runService = game:GetService("RunService")
local input = game:GetService("UserInputService")
local players = game:GetService("Players")

CanToggleMouse = {allowed = true; activationkey = Enum.KeyCode.P;} -- lets you move your mouse around in firstperson
CanViewBody = false
Sensitivity = 0.6
Smoothness = 0.1
FieldOfView = 90

local cam = game.Workspace.CurrentCamera
local player = players.LocalPlayer
local mouse = player:GetMouse()

local character = player.Character or player.CharacterAdded:wait()
local humanoidpart = character.HumanoidRootPart
local char = character
local head = character:WaitForChild("Head")
local CamPos,TargetCamPos = cam.CoordinateFrame.p,cam.CoordinateFrame.p 
local AngleX,TargetAngleX = 0,0
local AngleY,TargetAngleY = 0,0

local running = true

---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- 
game.Players.LocalPlayer:GetMouse().KeyDown:Connect(function(key)
	if key == "p" then
		firstpersonTOG = not firstpersonTOG
	end
end)
local speedteller = Instance.new("TextLabel")
speedteller.Size = UDim2.new(0.162, 0,0.085, 0)
speedteller.Position = UDim2.new(0.418, 0,0.914, 0)
local lockmouse = Instance.new("BoolValue")
lockmouse.Value = false
lockmouse.Changed:connect(function()
	if lockmouse.Value == true then
		game:GetService("UserInputService").MouseBehavior = Enum.MouseBehavior.LockCenter 
	end
	if lockmouse.Value == false then
		game:GetService("UserInputService").MouseBehavior = Enum.MouseBehavior.Default
	end
end)
game["Run Service"].RenderStepped:Connect(function()

	--if workspace.CurrentCamera.CFrame.Position.Y <= game.Players.LocalPlayer.Character.Head.CFrame.Position.Y + 1 and workspace.CurrentCamera.CFrame.Position.Z <= game.Players.LocalPlayer.Character.Head.CFrame.Position.Z + 1 and workspace.CurrentCamera.CFrame.Position.X <= game.Players.LocalPlayer.Character.Head.CFrame.Position.X + 1 then
	speedteller.Text = "animation speed = "..animspeed
	if animspeed < 1 then
		animspeed = 1
	end
	if animspeed > 5 then
		animspeed = 5
	end
	if animspeed == 5 then
		animationspeed = 0.25
	end
	if animspeed == 4 then
		animationspeed = 0.395
	end
	if animspeed == 3 then
		animationspeed = 0.475
	end
	if animspeed == 2 then
		animationspeed = 0.525
	end
	if animspeed == 1 then
		animationspeed = 0.6
	end

	if firstpersonTOG == true and ch ~= nil then
		if ch ~= nil then
			cam.CoordinateFrame = CFrame.new(ch["Head"].Position) 
			for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
				if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
				else
					if v:IsA("Accessory") then
						v.Handle.LocalTransparencyModifier = 1	
					end
				end
			end

			CamPos = CamPos + (TargetCamPos - CamPos) *0.28 
			AngleX = AngleX + (TargetAngleX - AngleX) *0.35 
			local dist = TargetAngleY - AngleY 
			dist = math.abs(dist) > 180 and dist - (dist / math.abs(dist)) * 360 or dist 
			AngleY = (AngleY + dist *0.35) %360
			cam.CameraType = Enum.CameraType.Scriptable
			--game.Players.LocalPlayer.CameraMode = Enum.CameraMode.LockFirstPerson
			if lockmouse.Value == false then
				lockmouse.Value = true
			end
			cam.CameraSubject = ch["Head"]
			cam.CoordinateFrame = CFrame.new(ch["Head"].Position) 
				* CFrame.Angles(0,math.rad(AngleY),0) 
				* CFrame.Angles(math.rad(AngleX),0,0)
				* CFrame.new(0,0.5,0) -- offset

			humanoidpart.CFrame=CFrame.new(humanoidpart.Position)*CFrame.Angles(0,math.rad(AngleY),0)	
		else
			cam.CameraSubject = game.Players.LocalPlayer.Character:FindFirstChildOfClass("Humanoid")
			cam.CameraType = Enum.CameraType.Follow
			lockmouse.Value = false	
		end

	else
		cam.CameraType = Enum.CameraType.Follow
		--game.Players.LocalPlayer.CameraMode = Enum.CameraMode.Classic
		if lockmouse.Value == true then
			lockmouse.Value = false
		end

		cam.CameraSubject = game.Players.LocalPlayer.Character:FindFirstChildOfClass("Humanoid")
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
			else
				if v:IsA("Accessory") then
					v.Handle.LocalTransparencyModifier = 0	
				end
			end
		end
	end
	if clientavatar == true then
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == "DefaultCharacter6" then
				for ia,va in pairs(v:GetDescendants()) do
					if va:IsA('BasePart') then
						va.LocalTransparencyModifier = va.Transparency

					end

				end
			end
		end
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
				v.Handle.LocalTransparencyModifier = 1
			end
		end
	else
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
				v.Handle.LocalTransparencyModifier = 0
			end
		end
	end
end)

---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- 

input.InputChanged:connect(function(inputObject)

	if inputObject.UserInputType == Enum.UserInputType.MouseMovement then
		local delta = Vector2.new(inputObject.Delta.x/Sensitivity,inputObject.Delta.y/Sensitivity) * Smoothness

		local X = TargetAngleX - delta.y 
		TargetAngleX = (X >= 80 and 80) or (X <= -80 and -80) or X 
		TargetAngleY = (TargetAngleY - delta.x) %360 
	end	

end)


print"test3"
---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- 
--welds

neck = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
neck.Part0 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")
neck.Part1 = ch.Head
neck.C0 = CFrame.new(0,1,0)
neck.C1 = CFrame.new(0,-0.5,0)

torso = Instance.new("Weld",root)
torso.Part0 = root
torso.Part1 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")


rs = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
rs.Part0 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")
rs.Part1 = ch["Right Arm"]
rs.C0 = CFrame.new(1.5,0.5,0)
rs.C1 = CFrame.new(0,0.5,0)

ls = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
ls.Part0 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")
ls.Part1 = ch["Left Arm"]
ls.C0 = CFrame.new(-1.5,0.5,0)
ls.C1 = CFrame.new(0,0.5,0)

rh = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
rh.Part0 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")
rh.Part1 = ch["Right Leg"]
rh.C0 = CFrame.new(0.5,-1,0)
rh.C1 = CFrame.new(0,1,0)

lh = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
lh.Part0 = ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso")
lh.Part1 = ch["Left Leg"]
lh.C0 = CFrame.new(-0.5,-1,0)
lh.C1 = CFrame.new(0,1,0)
--ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso") --sexx
--	ch[headhat].Handle:BreakJoints()
standhead = ch[headhat].Handle:FindFirstChildOfClass("Weld") --Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standhead.Part0 = ch:FindFirstChild(torsohat).Handle --sexx
standhead.Part1 = ch[headhat].Handle
standhead.C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(-5),math.rad(0),math.rad(0))
standhead.C1 = CFrame.new(0, 0, 0)
local standheadorigin = standhead.C0
--ch[leftarmhat].Handle:BreakJoints()

standleftarm = ch[leftarmhat].Handle:FindFirstChildOfClass("Weld") --Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standleftarm.Part0 = ch:FindFirstChild(torsohat).Handle --sexx
standleftarm.Part1 = ch[leftarmhat].Handle

standleftarm.C0 = CFrame.new(-1, -0.05, 0.5)*CFrame.Angles(math.rad(45),math.rad(-140),math.rad(-200))
standleftarm.C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))
local standleftarmorigin = standleftarm.C0
--ch[rightleghat].Handle:BreakJoints()

standrightleg = ch[rightleghat].Handle:FindFirstChildOfClass("Weld") -- Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standrightleg.Part0 = ch:FindFirstChild(torsohat).Handle --sexx
standrightleg.Part1 = ch[rightleghat].Handle

standrightleg.C0 = CFrame.new(0.5, -2, 0.1)*CFrame.Angles(math.rad(85),math.rad(-180),math.rad(-180))
standrightleg.C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))
local standrightlegorigin = standrightleg.C0
--ch[leftleghat].Handle:BreakJoints()

standleftleg = ch[leftleghat].Handle:FindFirstChildOfClass("Weld") -- Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standleftleg.Part0 = ch:FindFirstChild(torsohat).Handle --sexx
standleftleg.Part1 = ch[leftleghat].Handle

standleftleg.C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(70),math.rad(-180),math.rad(-180))
standleftleg.C1 = CFrame.new(0,0,0)
local standleftlegorigin = standleftleg.C0
--ch[rightarmhat].Handle:BreakJoints()

standrightarm = ch[rightarmhat].Handle:FindFirstChildOfClass("Weld") --Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standrightarm.Part0 = ch:FindFirstChild(torsohat).Handle --sexx
standrightarm.Part1 = ch[rightarmhat].Handle
--ch[rightarmhat].Handle:FindFirstChildOfClass("SpecialMesh"):Destroy()

standrightarm.C0 = CFrame.new(1, -0.05, 0.5)*CFrame.Angles(math.rad(45),math.rad(140),math.rad(-200))
standrightarm.C1 = CFrame.new(0,0,0)
local standrightarmorigin = standrightarm.C0
--ch[torsohat].Handle:BreakJoints()

standtorso = ch[torsohat].Handle:FindFirstChildOfClass("Weld") --Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standtorso.Part0 =ch['HumanoidRootPart']
standtorso.Part1 = ch[torsohat].Handle

standtorso.C0 = CFrame.new(2, 2.5, 3)*CFrame.Angles(math.rad(5),math.rad(-0),math.rad(0))
standtorso.C1 = CFrame.new(0, 0, 0)
local standtorsoorigin = standtorso.C0
ch[hairaccessory].Handle:BreakJoints()
standhair = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standhair.Part0 =ch[headhat].Handle
standhair.Part1 = ch[hairaccessory].Handle
standhair.C0 = CFrame.new(0, -0.65, 0.05)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))
standhair.C1 = CFrame.new(0,0,0)
ch[cheek1hat].Handle:BreakJoints()
standass1 = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))
standass1.Part1 =ch[cheek1hat].Handle

standass1.Part0 = ch[torsohat].Handle
standass1.C0 = CFrame.new(0.5, -1, 0.5)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))
standass1.C1 = CFrame.new(0,0,0)

ch[cheek2hat].Handle:BreakJoints()
standass2 = Instance.new("Weld",ch:FindFirstChild("Torso") or ch:FindFirstChild("LowerTorso"))

standass2.Part0 = ch[torsohat].Handle
standass2.Part1 = ch[cheek2hat].Handle

standass2.C0 = CFrame.new(-0.5, -1, 0.5)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))
standass2.C1 = CFrame.new(0,0,0)

local standass2origin = standass2.C0
local standass1origin = standass1.C0



--[[for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
	if v:IsA("SpecialMesh") then
		if v.Parent:IsA("BasePart") then
			if v ~= nil then
				if v.Parent.Size == Vector3.new(2, 2, 2) then
					if v ~= nil then
 v:Destroy()
					end
	       
		end	
			end
			if v ~= nil then
				if v.Parent.Size == Vector3.new(1, 1, 2) then
					if v ~= nil then
	v:Destroy()
					end
		
		end
			end
			if v ~= nil then
				if v.Parent.Size == Vector3.new(2, 2, 1) then
					if v ~= nil then
v:Destroy()
					end
			
		end	
			end
		
		
		
		end
	
	end
	end]]--

local cf, v3 = CFrame.new, Vector3.new
local v3_0, v3_101, v3_010, v3_d, v3_u = v3(0, 0, 0), v3(1, 0, 1), v3(0, 1, 0), v3(0, -10000, 0), v3(0, 10000, 0)
local vel, cfr, raycastresult, onground = v3_0, HumanoidRootPart.CFrame, nil, true

---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- 
print"test3"


print"test4"


print"test5"





local ContextActionService =game:GetService("ContextActionService")
local FowardMovement = Enum.PlayerActions.CharacterForward
local BackwardMovement = Enum.PlayerActions.CharacterBackward

local function Sink()
	return Enum.ContextActionResult.Sink
end

ContextActionService:BindAction("SinkBackwardMovement",Sink,false,BackwardMovement)

ContextActionService:UnbindAction("SinkBackwardMovement")
if game.Players.LocalPlayer.PlayerGui:FindFirstChild('GUI') then
	game.Players.LocalPlayer.PlayerGui.GUI:destroy()
end





local scrollchange = true

local UIS = game:GetService("UserInputService")

UIS.InputChanged:Connect(function(input)
	if input.UserInputType == Enum.UserInputType.MouseWheel then
		if input.Position.Z > 0 then
			animspeed += 1
		else
			animspeed -= 1
		end
	end


end)







local LookPart = Instance.new("Part",game.Players.LocalPlayer.Character)
LookPart.Size = Vector3.new(1, 1, 1)
LookPart.CanCollide = false
LookPart.Name = "LookPart"
LookPart.Transparency = 1
LookPart.Anchored = true



local player = game.Players.LocalPlayer
local character = player.Character or player.CharacterAdded:Wait()
local root = character:WaitForChild("HumanoidRootPart")





local standanimating = false

function tweenobject(object,stuff,edirection,estyle,speed,waitthing)
	if standanimating == false then
		local speedthing = 1
		local tween = tweenservice:Create(object,TweenInfo.new(speed/speedthing,estyle,edirection,0,false,0),stuff)
		tween:Play()
		if waitthing == true then
			tween.Completed:Wait()
			tween:Destroy()
		end	
	end

end
function tweenobject3(object,stuff,edirection,estyle,speed,waitthing)

	local speedthing = 1
	local tween = tweenservice:Create(object,TweenInfo.new(speed/speedthing,estyle,edirection,0,false,0),stuff)
	tween:Play()
	if waitthing == true then
		tween.Completed:Wait()
		tween:Destroy()
	end	


end
local animationnumbervalue = Instance.new('NumberValue')
animationnumbervalue.Value = 0
animationnumbervalue.Parent = ch
animationnumbervalue.Name = "AnimationValue"

function tweenobject2(object,stuff,edirection,estyle,speed,waitthing)
	if thingysomething == true then
		local speedthing = 1
		local tween = tweenservice:Create(object,TweenInfo.new(speed/speedthing,estyle,edirection,0,false,0),stuff)
		tween:Play()
		if waitthing == true then
			tween.Completed:Wait()
			tween:Destroy()
		end	
	end

end

function createsound(id,volume,timeposition)
	sfx = Instance.new('Sound',game:GetService("Players").LocalPlayer.Character:FindFirstChild("Torso") or game:GetService("Players").LocalPlayer.Character:FindFirstChild("LowerTorso"))
	sfx.Volume = volume
	sfx.SoundId = id
	sfx.TimePosition = timeposition
	sfx.PlayOnRemove = true 
	sfx:destroy()
end

local music = Instance.new("Sound",game.Players.LocalPlayer.Character.HumanoidRootPart)
music.SoundId = "rbxassetid://9042666762"
music.Volume = 0.5
music.PlayOnRemove = false
music.Looped = true
music:Play()

ch.Humanoid.WalkSpeed = 16
holdingshift = false


mememode = false


local UIS = game:GetService("UserInputService")

UIS.InputBegan:Connect(function(Input, GameProcessedEvent)
	if Input.KeyCode == Enum.KeyCode.LeftControl then
		--if holdingbutton2 == false then
		if anim ~= 'idle' then
			holdingshift = true
			ch.Humanoid.WalkSpeed = 60
		end
		--end
	end
end)


UIS.InputEnded:Connect(function(Input, GameProcessedEvent)
	if Input.KeyCode == Enum.KeyCode.LeftControl then
		--if holdingbutton2 == false then
		holdingshift = false
		ch.Humanoid.WalkSpeed = 16
		--end
	end
end)











mouse = game.Players.LocalPlayer:GetMouse()

local musictoggle = true


mouse.KeyDown:connect(function(key)

	if key == "q" then
		if  animmode == "bang" then
			animmode = ""
		else
			animmode = "bang"	
		end

	end
	if key == "f" then
		if animmode == "metroboominmakeitboom" then
			animmode = ""
		else
			animmode = "metroboominmakeitboom"	
		end

	end
	if key == "c" then
		if animmode == "nelson" then
			animmode = ""
		else
			animmode = "nelson"	
		end

	end
	if key == "l" then
		standrape.Value = not standrape.Value
		print(standrape.Value)
		if standrape.Value == false then
			standtorso.Part0 =ch['HumanoidRootPart']
		end
	end
	if key == "m" then
		musictoggle = not musictoggle

	end
	if key == "y" then
		if  animmode == "penis" then
			animmode = ""
		else
			animmode = "penis"	
		end

	end
	if key == "e" then
		if animmode == "ride" then
			animmode = ""
		else
			animmode = "ride"	
		end

	end
	if key == "r" then
		if animmode == "suck" then
			animmode = ""
		else
			animmode = "suck"	
		end

	end
	if key == "t" then
		if animmode == "side" then
			animmode = ""
		else
			animmode = "side"	
		end

	end
	if key == "b" then
		if animmode == "headsit" then
			animmode = ""
		else
			animmode = "headsit"	
		end

	end
	if key == "z" then

		animmode = "lightdash"	
		LightDash()

	end
	if key == "u" then

		animmode = "barrage"	


	end
	if key == "x" then

		armshooting = not armshooting


	end
	if key == "v" then
		clientavatar = not clientavatar
		--if game["Run Service"]:IsStudio() then
		mommycreate()
		--end
	end
end)
mouse.KeyUp:Connect(function(key)
	if key == "z" then
		animmode = ""	
	end
	if key == "u" then

		animmode = ""	


	end
end)
mouse.Button1Down:Connect(function()
	if standrape.Value == false then
		local randomnum = math.random(1,3)
		if randomnum == 1 then
			animmode = "punch"	
		end
		if randomnum == 2 then
			animmode = "punch2"	
		end
		if randomnum == 3 then
			animmode = "kick"	
		end
	elseif standrape.Value == true then
		if mouse.Target ~= nil then
			if mouse.Target.Parent:FindFirstChildOfClass("Humanoid") then
				standtorso.Part0 = mouse.Target.Parent.HumanoidRootPart
			end
		end
	end

end)

function mommycreate()
	if clientavatar == true then
		local modellOL = Instance.new('ObjectValue')
		if game["Run Service"]:IsStudio() then
			modellOL.Value = workspace.supermanguy:Clone()
		else
			modellOL.Value = game:GetObjects('rbxassetid://11902798575')[1]
		end
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
				v.Handle.Transparency = 1
			end
		end
		--	
		local mommy = modellOL.Value.DefaultCharacter6
		mommy.Parent = game.Players.LocalPlayer.Character:FindFirstChild("HumanoidRootPart")
		--mommy.explos:Destroy()
		for i,v in pairs(mommy:GetDescendants()) do
			if v:IsA("BasePart") then
				v.Anchored = false		
				v.CanCollide = false
			end
			if v.Name == "torsoweld" then
				v.Part0 = standtorso.Part1
			end
			if v.Name == "Head" then
				local weld = Instance.new("Weld")
				weld.Parent = v
				weld.Part1 = v
				weld.Part0 = standhead.Part1
			end
			if v.Name == "Joint" then
				v.Part0 = standhair.Part1
			end
			if v.Name == "rightlegweld" then
				v.Part0 = standrightleg.Part1
			end
			if v.Name == "leftlegweld" then
				v.Part0 = standleftleg.Part1
			end
			if v.Name == "leftarmweld" then
				v.Part0 = standleftarm.Part1
			end
			if v.Name == "rightarmweld" then
				v.Part0 = standrightarm.Part1
			end
			if v.Name == "cheek1weld" then
				v.Part0 = standass2.Part1
			end
			if v.Name == "cheek2weld" then
				v.Part0 = standass1.Part1
			end
			if v.Name == "G-String" then
				v.Transparency = 0
			end
		end		

	elseif clientavatar == false then
		for i,v in pairs(game.Players.LocalPlayer.Character.Model:GetDescendants()) do
			if v.Name == headhat or v.Name == torsohat or v.Name == leftarmhat or v.Name == rightleghat or v.Name == rightarmhat or v.Name == leftleghat or v.Name == hairaccessory or v.Name == cheek1hat or v.Name == cheek2hat then
				v.Handle.Transparency = 0
			end
		end
		for i,v in pairs(game.Players.LocalPlayer.Character:GetDescendants()) do
			if v.Name == "DefaultCharacter6" then
				v:Destroy()
			end
		end
	end

end





HB:connect(function()

	if animspeed ~= 0 and animmode == "punch" then
		fling(ch['Kate Hair'].Handle,1)
	end
	if animspeed ~= 0 and animmode == "punch2" then
		fling(ch['Robloxclassicred'].Handle,1)
	end
	if animspeed ~= 0 and animmode == "kick" then
		fling(ch['Pink Hair'].Handle,1)
	end
	if animspeed ~= 0 and animmode == "barrage" then
		local randoma = math.random(1,2)
		if randoma == 1 then
			fling(ch['Kate Hair'].Handle,0.25)	
		end
		if randoma == 2 then
			fling(ch['Robloxclassicred'].Handle,0.25)	
		end
	end
	if animspeed ~= 0 and animmode == "gunshot" then
		fling(ch['Kate Hair'].Handle,1)
	end
end)




if game["Run Service"]:IsStudio() then
else
	game["Run Service"].RenderStepped:connect(
		function()
			setscriptable(game.Players.LocalPlayer, "SimulationRadius", true)
			game.Players.LocalPlayer.SimulationRadius = math.huge * 1/0
		end)

	local LocalPlayer = game:GetService("Players").LocalPlayer
	LocalPlayer.SimulationRadiusChanged:Connect(function(radius)
		radius = 9e9
		return radius
	end)
	print(gethiddenproperty(game.Players.LocalPlayer, "SimulationRadius")) --//test if you have over 1000 (will bypass the limit)

end



Character = game.Players.LocalPlayer.Character
local function rawrbow(Part)
	local Selection = Instance.new("Highlight",Part)
	Selection.Name = "RAINBOWHRP"
	Selection.Adornee = Part
	Selection.FillTransparency = 0.7
	Selection.DepthMode = 1
	task.spawn(function()
		while Character do
			for i = 0,1,0.004 do
				Selection.FillColor = Color3.fromHSV(i,1,1)
				Selection.OutlineColor = Color3.fromHSV(i,1,1)
				--if isnetworkowner(Part) then Selection.Enabled = true else Selection.Enabled = false end
				task.wait()
				Part.Color = Color3.fromRGB(i,0,0)
			end
		end
	end)
end
if game.Players.LocalPlayer.Character.Model:FindFirstChild("HumanoidRootPart") then
	rawrbow(game.Players.LocalPlayer.Character.Model['HumanoidRootPart'])	
end







local torsoc0 = Instance.new('CFrameValue')
torsoc0.Value = CFrame.new(2, 2.5, 3, 0.718977332, -0.419666231, -0.554032385, 0.330366105, 0.907673359, -0.258819044, 0.611498058, 0.00305151939, 0.791240036)
if torsoc0.Value == CFrame.new(2, 2.5, 3, 0.718977332, -0.419666231, -0.554032385, 0.330366105, 0.907673359, -0.258819044, 0.611498058, 0.00305151939, 0.791240036) then
	tweenobject2(torsoc0,{Value = CFrame.new(2, 2.25, 3, 0.721101463, -0.358820856, -0.59267211, 0.320622027, 0.931153953, -0.173648193, 0.614177644, -0.064805761, 0.786502421)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if torsoc0.Value == CFrame.new(2, 2.25, 3, 0.721101463, -0.358820856, -0.59267211, 0.320622027, 0.931153953, -0.173648193, 0.614177644, -0.064805761, 0.786502421) then
	tweenobject2(torsoc0,{Value = CFrame.new(2, 2.5, 3, 0.718977332, -0.419666231, -0.554032385, 0.330366105, 0.907673359, -0.258819044, 0.611498058, 0.00305151939, 0.791240036)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standtorsoorigin = torsoc0.Value

torsoc0.Changed:Connect(function()
	if torsoc0.Value == CFrame.new(2, 2.5, 3, 0.718977332, -0.419666231, -0.554032385, 0.330366105, 0.907673359, -0.258819044, 0.611498058, 0.00305151939, 0.791240036) then
		tweenobject2(torsoc0,{Value = CFrame.new(2, 2.25, 3, 0.721101463, -0.358820856, -0.59267211, 0.320622027, 0.931153953, -0.173648193, 0.614177644, -0.064805761, 0.786502421)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if torsoc0.Value == CFrame.new(2, 2.25, 3, 0.721101463, -0.358820856, -0.59267211, 0.320622027, 0.931153953, -0.173648193, 0.614177644, -0.064805761, 0.786502421) then
		tweenobject2(torsoc0,{Value = CFrame.new(2, 2.5, 3, 0.718977332, -0.419666231, -0.554032385, 0.330366105, 0.907673359, -0.258819044, 0.611498058, 0.00305151939, 0.791240036)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standtorsoorigin = torsoc0.Value
end)




local leftarmc0 = Instance.new('CFrameValue')
leftarmc0.Value = CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416)
if leftarmc0.Value == CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416) then
	tweenobject2(leftarmc0,{Value = CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if leftarmc0.Value == CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416) then
	tweenobject2(leftarmc0,{Value = CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standleftarmorigin = leftarmc0.Value

leftarmc0.Changed:Connect(function()
	if leftarmc0.Value == CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416) then
		tweenobject2(leftarmc0,{Value = CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if leftarmc0.Value == CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416) then
		tweenobject2(leftarmc0,{Value = CFrame.new(-1, -0.0500000007, 0.5, -0.766044617, 0.582563281, -0.271653712, 0, 0.42261824, 0.906307817, 0.642787457, 0.69427222, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standleftarmorigin = leftarmc0.Value
end)





local rightarmc0 = Instance.new('CFrameValue')
rightarmc0.Value = CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416)
if rightarmc0.Value == CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416) then
	tweenobject2(rightarmc0,{Value = CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if rightarmc0.Value == CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416) then
	tweenobject2(rightarmc0,{Value = CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standrightarmorigin = rightarmc0.Value

rightarmc0.Changed:Connect(function()
	if rightarmc0.Value == CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416) then
		tweenobject2(rightarmc0,{Value = CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if rightarmc0.Value == CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416) then
		tweenobject2(rightarmc0,{Value = CFrame.new(1, -0.0500000007, 0.5, 0.766044676, 0.582563221, 0.271653712, -3.69464601e-08, -0.42261824, 0.906307817, 0.642787397, -0.69427228, -0.323744416)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standrightarmorigin = rightarmc0.Value
end)




local headc0 = Instance.new('CFrameValue')
headc0.Value = CFrame.new(0, 1.60000002, -0.0500000007, 0.806707203, 0.142244279, 0.57357651, -0.173648208, 0.98480773, 0, -0.564862609, -0.0996005312, 0.819151998)
if headc0.Value == CFrame.new(0, 1.60000002, -0.0500000007, 0.806707203, 0.142244279, 0.57357651, -0.173648208, 0.98480773, 0, -0.564862609, -0.0996005312, 0.819151998) then
	tweenobject2(headc0,{Value = CFrame.new(0, 1.60000002, -0.0500000007, 0.787969887, 0.131769687, 0.601448417, -0.190692782, 0.981029212, 0.0348994955, -0.585439801, -0.142191619, 0.79814899)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if headc0.Value == CFrame.new(0, 1.60000002, -0.0500000007, 0.787969887, 0.131769687, 0.601448417, -0.190692782, 0.981029212, 0.0348994955, -0.585439801, -0.142191619, 0.79814899) then
	tweenobject2(headc0,{Value = CFrame.new(0, 1.60000002, -0.0500000007, 0.806707203, 0.142244279, 0.57357651, -0.173648208, 0.98480773, 0, -0.564862609, -0.0996005312, 0.819151998)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standheadorigin = headc0.Value

headc0.Changed:Connect(function()
	if headc0.Value == CFrame.new(0, 1.60000002, -0.0500000007, 0.806707203, 0.142244279, 0.57357651, -0.173648208, 0.98480773, 0, -0.564862609, -0.0996005312, 0.819151998) then
		tweenobject2(headc0,{Value = CFrame.new(0, 1.60000002, -0.0500000007, 0.787969887, 0.131769687, 0.601448417, -0.190692782, 0.981029212, 0.0348994955, -0.585439801, -0.142191619, 0.79814899)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if headc0.Value == CFrame.new(0, 1.60000002, -0.0500000007, 0.787969887, 0.131769687, 0.601448417, -0.190692782, 0.981029212, 0.0348994955, -0.585439801, -0.142191619, 0.79814899) then
		tweenobject2(headc0,{Value = CFrame.new(0, 1.60000002, -0.0500000007, 0.806707203, 0.142244279, 0.57357651, -0.173648208, 0.98480773, 0, -0.564862609, -0.0996005312, 0.819151998)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standheadorigin = headc0.Value
end)





local leftlegc0 = Instance.new('CFrameValue')
leftlegc0.Value = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)
if leftlegc0.Value == CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452) then
	tweenobject2(leftlegc0,{Value = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if leftlegc0.Value == CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452) then
	tweenobject2(leftlegc0,{Value = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standleftlegorigin = leftlegc0.Value

leftlegc0.Changed:Connect(function()
	if leftlegc0.Value == CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452) then
		tweenobject2(leftlegc0,{Value = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if leftlegc0.Value == CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452) then
		tweenobject2(leftlegc0,{Value = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standleftlegorigin = leftlegc0.Value
end)

local rightlegc0 = Instance.new('CFrameValue')
rightlegc0.Value = CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)
if rightlegc0.Value == CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567) then
	tweenobject2(rightlegc0,{Value = CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
end
if rightlegc0.Value == CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567) then
	tweenobject2(rightlegc0,{Value =CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
end
standrightlegorigin = rightlegc0.Value

rightlegc0.Changed:Connect(function()
	if rightlegc0.Value == CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567) then
		tweenobject2(rightlegc0,{Value = CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)	
	end
	if rightlegc0.Value == CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567) then
		tweenobject2(rightlegc0,{Value =CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)},Enum.EasingDirection.InOut,Enum.EasingStyle.Quart,5,false)
	end
	standrightlegorigin = rightlegc0.Value
end)


function LightDash()

	local spd = Vector3.new(game.Players.LocalPlayer.Character:FindFirstChild('HumanoidRootPart').Velocity.x,0,game.Players.LocalPlayer.Character:FindFirstChild('HumanoidRootPart').Velocity.z).magnitude + 10
	spd = spd + 30
	local dir = Vector3.new(game.Players.LocalPlayer.Character:FindFirstChild('HumanoidRootPart').Velocity.x,0,game.Players.LocalPlayer.Character:FindFirstChild('HumanoidRootPart').Velocity.z).unit
	local GravPoint = game.Players.LocalPlayer.Character:FindFirstChild('HumanoidRootPart').Velocity.y
	if spd > 40 then

		local NV = Vector3.new(0,0,0)
		local bv = Instance.new("BodyVelocity", game.Players.LocalPlayer.Character:FindFirstChild('Torso'))
		bv.maxForce = Vector3.new(1/0,1/0,1/0)
		bv.velocity = dir*spd
		local bg = Instance.new("BodyGyro", game.Players.LocalPlayer.Character:FindFirstChild('Torso'))
		bg.maxTorque = Vector3.new(1/0,1/0,1/0)
		bg.cframe = CFrame.new(NV, dir) * CFrame.Angles(math.pi/2.2,0.24,0)


		Humanoid.PlatformStand = true
		Humanoid.Sit = true

		while spd > 2 and animspeed ~= 0 and animmode == "lightdash" do
			swait()
			bv.velocity = dir*spd + Vector3.new(0,0,0)
			bg.cframe = CFrame.new(NV, dir) * CFrame.Angles(math.pi/2.2,0.24,0)

		end	
		bv:Destroy()
		bg:Destroy()

		Humanoid.PlatformStand = false
		Humanoid.Sit = false
		sliding = false
		wait(0.05)


	end
end
local RunService = game:GetService("RunService") 
if RunService:IsStudio() then
	if game.Players.LocalPlayer.PlayerGui:FindFirstChild("FunnyGUI") then
		game.Players.LocalPlayer.PlayerGui:FindFirstChild("FunnyGUI"):Destroy()
	end
	local guifunny = Instance.new("ScreenGui",game.Players.LocalPlayer.PlayerGui)
	guifunny.Name = "FunnyGUI"
	local info = Instance.new("TextLabel",guifunny)
	speedteller.Parent = guifunny
	speedteller.BackgroundTransparency = 1
	speedteller.TextColor3 = Color3.new(1,1,1)
	speedteller.TextScaled = true
	speedteller.Font = Enum.Font.DenkOne
	info.BackgroundTransparency = 1
	info.TextColor3 = Color3.new(1,1,1)
	info.Text = controlslist
	info.Position = UDim2.new(0.899,0,0.6,0)
	info.Size = UDim2.new(0,112,0,240)
	info.TextScaled = true
	info.TextYAlignment = Enum.TextYAlignment.Top
	info.FontFace.Weight = Enum.FontWeight.Bold
else
	if game.CoreGui:FindFirstChild("FunnyGUI") then
		game.CoreGui:FindFirstChild("FunnyGUI"):Destroy()	
	end
	local guifunny = Instance.new("ScreenGui",game.CoreGui)
	local info = Instance.new("TextLabel",guifunny)
	speedteller.Parent = guifunny
	speedteller.BackgroundTransparency = 1
	speedteller.TextColor3 = Color3.new(1,1,1)
	speedteller.TextScaled = true
	speedteller.Font = Enum.Font.DenkOne
	guifunny.Name = "FunnyGUI"
	info.BackgroundTransparency = 1
	info.TextColor3 = Color3.new(1,1,1)
	info.Text = controlslist
	info.Position = UDim2.new(0.899,0,0.6,0)
	info.Size = UDim2.new(0,112,0,240)
	info.TextScaled = true
	info.TextYAlignment = Enum.TextYAlignment.Top
	info.FontFace.Weight = Enum.FontWeight.Bold
end


game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false

if game.Players.LocalPlayer.Character.Humanoid.RigType == Enum.HumanoidRigType.R15 then 
	local RunService = game:GetService("RunService") 
	if RunService:IsStudio() then

	else
		--game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
		--loadstring(game:HttpGet("https://pastebin.com/raw/3FfXxBQ7"))()
		--game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
	end 
else
	local RunService = game:GetService("RunService") 
	if RunService:IsStudio() then

	else
		game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
		loadstring(game:HttpGet("https://pastebin.com/raw/3FfXxBQ7"))()
		game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
	end 
	--	loadstring(game:HttpGet("https://raw.githubusercontent.com/Tescalus/Pendulum-Hubs-Source/main/Reanimation.lua"))()
end




while true do 
	game:GetService("RunService").Heartbeat:Wait()
	if animmode == "penis" then
		standass1.Part0 = ch[rightleghat].Handle
		standass2.Part0 =ch['Pal Hair'].Handle
	else
		standass1.Part0 = ch[torsohat].Handle
		standass2.Part0 = ch['Star ManAccessory'].Handle
		tweenobject2(standass1,{C0 = standass1origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standass2,{C0 = standass2origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
	end
	if animmode == "bang" then

	else
		tweenobject2(standrightarm,{C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standleftarm,{C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)

	end
	sine = sine + change
	LookPart.CFrame = CFrame.new(ch:FindFirstChild("Torso").Position or ch:FindFirstChild("UpperTorso").Position)*CFrame.Angles(game.Workspace.CurrentCamera.CFrame:ToEulerAnglesXYZ())
	mpos = mouse.Hit.p
	mposx,mposy,mposz = mpos.X,mpos.Y,mpos.Z
	LPOR = game:GetService("Players").LocalPlayer.Character.LookPart.Orientation
	standtorsoorigin = torsoc0.Value
	--local rlegray = Ray.new(ch["Right Leg"].Position + Vector3.new(0, 0.5, 0), Vector3.new(0, -2, 0))
	--	local rlegpart, rlegendPoint = workspace:FindPartOnRay(rlegray, char)
	--	local llegray = Ray.new(ch["Left Leg"].Position + Vector3.new(0, 0.5, 0), Vector3.new(0, -2, 0))
	--	local llegpart, llegendPoint = workspace:FindPartOnRay(llegray, char)
	local rightvector = (Root.Velocity * Root.CFrame.rightVector).X + (Root.Velocity * Root.CFrame.rightVector).Z
	local lookvector = (Root.Velocity * Root.CFrame.lookVector).X + (Root.Velocity * Root.CFrame.lookVector).Z
	if lookvector > ch.Humanoid.WalkSpeed then
		lookvector = ch.Humanoid.WalkSpeed
	end
	if lookvector < -ch.Humanoid.WalkSpeed then
		lookvector = -ch.Humanoid.WalkSpeed
	end
	if rightvector > ch.Humanoid.WalkSpeed then
		rightvector = ch.Humanoid.WalkSpeed
	end
	if rightvector < -ch.Humanoid.WalkSpeed then
		rightvector = -ch.Humanoid.WalkSpeed
	end
	local lookvel = lookvector / ch.Humanoid.WalkSpeed
	local rightvel = rightvector / ch.Humanoid.WalkSpeed


	if hum.MoveDirection.Magnitude > 0 then

		anim = "walk"

	else 
		anim = "idle"
	end
	if hum:GetState() == Enum.HumanoidStateType.Freefall then
		anim = "jump"
	end

	music.Playing = musictoggle

	if anim == "jump"  then
		tweenobject(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject(torso,{C0 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject(rh,{C0 = CFrame.new(0.5,-0.5,-0.25)*CFrame.Angles(math.rad(-8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject(lh,{C0 = CFrame.new(-0.5,-0.75,-0.15)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
	end
	if root.Velocity.Y < -1 then
		tweenobject(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(-8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,1,false)
		tweenobject(torso,{C0 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(-8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,1,false)
		tweenobject(rh,{C0 = CFrame.new(0.5,-0.5,-0.25)*CFrame.Angles(math.rad(-8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,1,false)
		tweenobject(lh,{C0 = CFrame.new(-0.5,-0.75,-0.15)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,1,false)
	end

	if animspeed ~= 0 and animmode == "bang" then
		standanimating = true

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://9119990321",6,2)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0.75)*CFrame.Angles(math.rad(20),math.rad(60),math.rad(20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.1250505050505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.5, -2.25)*CFrame.Angles(math.rad(60),math.rad(-180),math.rad(-180))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.1250505050505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(0.75, 1.5, -0.125, -0.0871558785, -0.862729967, 0.49809733, -4.37113847e-08, -0.49999997, -0.866025448, 0.99619472, -0.0754792318, 0.0435778983)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.1250505050505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-0.75, 1.25, -0.5, -3.55205572e-08, 0.906307817, 0.42261824, 3.69464601e-08, -0.42261824, 0.906307817, 1, 4.78068038e-08, -1.84732301e-08)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.1250505050505050505,false)

			tweenobject2(standrightleg,{C0 = CFrame.new(1, -0.91, -1.1)*CFrame.Angles(math.rad(20),math.rad(-10),math.rad(-50))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.1250505050505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-1, -0.91, -1.1)*CFrame.Angles(math.rad(10),math.rad(10),math.rad(50))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			tweenobject2(standrightarm,{C1 = CFrame.new(0,0,-0.25)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C1 = CFrame.new(0,0,0.25)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then
				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(32.5),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(32.5),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -0.907, 0)*CFrame.Angles(math.rad(15),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -0.907, 0)*CFrame.Angles(math.rad(15),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end

		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	
			tweenobject3(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(0+10*math.cos(sine/70)+LPOR.X),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0.75)*CFrame.Angles(math.rad(15),math.rad(55),math.rad(20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.5, -2)*CFrame.Angles(math.rad(60),math.rad(-180),math.rad(-180))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(0.75, 1.35000002, -0.25, -0.0871556774, -0.862729967, 0.49809733, 7.54978942e-08, -0.49999997, -0.866025448, 0.99619472, -0.0754789934, 0.0435778983)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-0.75, 1.10000002, -0.649999976, 1.8056005e-07, 0.906307817, 0.422618151, -6.3813566e-08, -0.422618151, 0.906307817, 1, -1.90611772e-07, -1.84732265e-08)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)



			tweenobject2(standrightleg,{C0 = CFrame.new(1, -0.91, -1.1)*CFrame.Angles(math.rad(20),math.rad(-10),math.rad(-45))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-1, -0.91, -1.1)*CFrame.Angles(math.rad(20),math.rad(10),math.rad(45))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			tweenobject2(standrightarm,{C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(14),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C1 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(-14),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then
				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(16.25),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(16.25),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -0.907, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -0.907, 0)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end


		end


	end
	if animspeed ~= 0 and animmode == "metroboominmakeitboom" then

		if standrape.Value == false then
			standanimating = true
		end

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://9119990321",6,2)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.675, -0.2)*CFrame.Angles(math.rad(-25),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -0.5, -1.75)*CFrame.Angles(math.rad(-50),math.rad(180),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.125, -0.25)*CFrame.Angles(math.rad(-25),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.125, -0.25)*CFrame.Angles(math.rad(-25),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(1, -1, -1)*CFrame.Angles(math.rad(35),math.rad(-35),math.rad(-20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-1, -1, -1)*CFrame.Angles(math.rad(35),math.rad(35),math.rad(20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then

				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, -0.45, 0)*CFrame.Angles(math.rad(85),math.rad(20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, -0.45, 0)*CFrame.Angles(math.rad(85),math.rad(-20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(-10),math.rad(4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(10),math.rad(-4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.675, -0.25)*CFrame.Angles(math.rad(-35),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -0.65, -1.5)*CFrame.Angles(math.rad(-60),math.rad(180),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.125, -0.25)*CFrame.Angles(math.rad(-20),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.125, -0.25)*CFrame.Angles(math.rad(-20),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(1, -1, -1)*CFrame.Angles(math.rad(50),math.rad(-35),math.rad(-20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-1, -1, -1)*CFrame.Angles(math.rad(50),math.rad(35),math.rad(20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then

				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, -0.45, 0.1)*CFrame.Angles(math.rad(75),math.rad(20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, -0.45, 0.1)*CFrame.Angles(math.rad(75),math.rad(-20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(-10),math.rad(4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(10),math.rad(-4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end


	end
	if animspeed ~= 0 and animmode == "nelson" then

		if standrape.Value == false then
			standanimating = true
		end

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://9119990321",6,2)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0)*CFrame.Angles(math.rad(-20),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0.5, -1.25)*CFrame.Angles(math.rad(10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.125, -0.25)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.125, -0.25)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.7, 0.25, -0.75)*CFrame.Angles(math.rad(70),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, 0.25, -0.75)*CFrame.Angles(math.rad(70),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then

				tweenobject2(torso,{C0 = CFrame.new(0, 0, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(80),math.rad(20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(80),math.rad(-20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(-4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0)*CFrame.Angles(math.rad(-18),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0.25, -1.25)*CFrame.Angles(math.rad(8),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.125, -0.25)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.125, -0.25)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.7, 0.25, -0.75)*CFrame.Angles(math.rad(72),math.rad(-12),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, 0.25, -0.75)*CFrame.Angles(math.rad(72),math.rad(12),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then

				tweenobject2(torso,{C0 = CFrame.new(0, 0, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.25, 0)*CFrame.Angles(math.rad(85),math.rad(20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.25, 0)*CFrame.Angles(math.rad(85),math.rad(-20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(-4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end


	end
	if animspeed ~= 0 and animmode == "suck" then
		if standrape.Value == false then
			standanimating = true
		end

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://344167846",.125,0)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0.15)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.5, -2)*CFrame.Angles(math.rad(45),math.rad(-180),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 1, -0.5)*CFrame.Angles(math.rad(45),math.rad(10),math.rad(-10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 1, -0.5)*CFrame.Angles(math.rad(45),math.rad(-10),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.85, -0.05, -1)*CFrame.Angles(math.rad(40),math.rad(0),math.rad(-10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.85, -0.05, -1)*CFrame.Angles(math.rad(40),math.rad(0),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then


				tweenobject2(torso,{C0 = CFrame.new(0, 0, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.25, 0.5, -0.5, 0.841477871, 0.432556033, 0.323744357, 0.144543961, 0.397131234, -0.906307817, -0.520597875, 0.809433222, 0.271653771)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, 0)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			end
		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0.05)*CFrame.Angles(math.rad(5),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.5, -2)*CFrame.Angles(math.rad(50),math.rad(-180),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 1, -0.5)*CFrame.Angles(math.rad(55),math.rad(10),math.rad(-10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 1, -0.5)*CFrame.Angles(math.rad(55),math.rad(-10),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.85, 0, -1)*CFrame.Angles(math.rad(50),math.rad(0),math.rad(-10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.85, 0, -1)*CFrame.Angles(math.rad(50),math.rad(0),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then


				tweenobject2(torso,{C0 = CFrame.new(0, 0, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.35000002, 0.5, -0.5, 0.760786653, 0.447714299, 0.469846338, 0.186738223, 0.542327166, -0.819152057, -0.62155652, 0.710938215, 0.328989953)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, 0)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			end
		end


	end
	if animspeed ~= 0 and animmode == "penis" then
		tweenobject2(standhead,{C0 = CFrame.new(0,0,0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standtorso,{C0 = CFrame.new(0, -10, 1)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standrightarm,{C0 = CFrame.new(0, 2, 9)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standleftarm,{C0 = CFrame.new(0, 4, 9)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standrightleg,{C0 = CFrame.new(0, 6, 9)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standleftleg,{C0 = CFrame.new(0, 8, 9)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standass1,{C0 = CFrame.new(-0.5, 1, -4.5)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standass2,{C0 = CFrame.new(0.5, 1, -6.5)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)

	end
	if animspeed ~= 0 and animmode == "side" then
		if standrape.Value == false then
			standanimating = true
		end

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://9119990321",6,2)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.5, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.75, -2)*CFrame.Angles(math.rad(0),math.rad(-90),math.rad(90))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(0, 0.75, -1)*CFrame.Angles(math.rad(30),math.rad(40),math.rad(-140))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-0.75, 1.25, -0.5)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(1.5, -1.1, 0)*CFrame.Angles(math.rad(-10),math.rad(-90),math.rad(90))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.65, -1.9, -0.1)*CFrame.Angles(math.rad(-80),math.rad(10),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then


				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.25, 0.5, -0.75)*CFrame.Angles(math.rad(75),math.rad(45),math.rad(-20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.25, 0.5, -0.75)*CFrame.Angles(math.rad(75),math.rad(-45),math.rad(20))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -0.907, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -0.907, 0)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			end
		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.5, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1.75, -1.75)*CFrame.Angles(math.rad(0),math.rad(-90),math.rad(90))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(0, 0.75, -1)*CFrame.Angles(math.rad(40),math.rad(40),math.rad(-140))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-0.75, 1.25, -0.5)*CFrame.Angles(math.rad(10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(1.5, -1.1, 0)*CFrame.Angles(math.rad(0),math.rad(-90),math.rad(90))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.65, -1.9, -0.1)*CFrame.Angles(math.rad(-80),math.rad(10),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			if standrape.Value == false then



				tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.25, 0.5, -0.75)*CFrame.Angles(math.rad(55),math.rad(45),math.rad(-30))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.25, 0.5, -0.75)*CFrame.Angles(math.rad(55),math.rad(-45),math.rad(30))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -0.907, 0)*CFrame.Angles(math.rad(-20),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -0.907, 0)*CFrame.Angles(math.rad(-20),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			end
		end


	end
	if animmode == "headsit" then


		if animationnumbervalue.Value == 0 then
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)	
			tweenobject2(standhead,{C0 = CFrame.new(0, 1.6, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 1.5, 0.5)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, -0.1, -0.25)*CFrame.Angles(math.rad(-70),math.rad(20),math.rad(-140))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, -0.1, -0.25)*CFrame.Angles(math.rad(-70),math.rad(-20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.75, -1, -1)*CFrame.Angles(math.rad(-90),math.rad(-20),math.rad(90))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, -1, -1)*CFrame.Angles(math.rad(-90),math.rad(20),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)


			if animationnumbervalue.Value == 1 then

				tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)	
				tweenobject2(standhead,{C0 = CFrame.new(0, 1.6, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,7 * 0.9285714285714286,false)
				tweenobject2(standtorso,{C0 = CFrame.new(0, 1.45, 0.55)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
				tweenobject2(standrightarm,{C0 = CFrame.new(1.5, -0.1, -0.25)*CFrame.Angles(math.rad(-68),math.rad(22),math.rad(-140))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
				tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, -0.1, -0.25)*CFrame.Angles(math.rad(-68),math.rad(-22),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
				tweenobject2(standrightleg,{C0 = CFrame.new(0.75, -1, -1)*CFrame.Angles(math.rad(-88),math.rad(-21),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
				tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, -1, -1)*CFrame.Angles(math.rad(-88),math.rad(21),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.3 * 0.9285714285714286,false)
			end

		end
	end
	if animspeed ~= 0 and animmode == "ride" then
		if standrape.Value == false then
			standanimating = true
		end

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://9119990321",6,2)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -0.75, -1)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.25, 0.25, 0.98480773, -0.171010092, -0.0301536582, 0, 0.173647985, -0.984807789, 0.173648193, 0.969846308, 0.171009883)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.25, 0.25, 0.98480773, 0.171010092, 0.0301536582, 0, 0.173647985, -0.984807789, -0.173648193, 0.969846308, 0.171009883)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.75, -1, -0.75, 0.939692616, 0.296198159, -0.171010062, 0, 0.49999997, 0.866025448, 0.342020154, -0.813797712, 0.469846278)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, -1, -0.75, 0.939692616, -0.296198159, 0.171010062, 0, 0.49999997, 0.866025448, -0.342020154, -0.813797712, 0.469846278)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then


				tweenobject2(torso,{C0 = CFrame.new(0, -2.5, 0)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(10.25),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(10.25),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(2))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(-2))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end
		if animationnumbervalue.Value == 1 then

			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,animationspeed,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, -1, -1)*CFrame.Angles(math.rad(-10),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, -0.125, 0.25, 0.98480773, -0.171010092, -0.0301536582, 0, 0.173647985, -0.984807789, 0.173648193, 0.969846308, 0.171009883)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.25, -0.125, 0.25, 0.98480773, 0.171010092, 0.0301536582, 0, 0.173647985, -0.984807789, -0.173648193, 0.969846308, 0.171009883)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.699999988, -0.75, -0.75, 0.866025388, 0.469846338, -0.171010017, 0, 0.342020035, 0.939692676, 0.5, -0.813797712, 0.29619804)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.75, -0.75, -0.75, 0.866025388, -0.469846338, 0.171010017, 0, 0.342020035, 0.939692676, -0.5, -0.813797712, 0.29619804)},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			if standrape.Value == false then


				tweenobject2(torso,{C0 = CFrame.new(0, -2.5, 0)*CFrame.Angles(math.rad(90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(10.25),math.rad(8),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(10.25),math.rad(-8),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(rh,{C0 = CFrame.new(0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(2.05))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)
				tweenobject2(lh,{C0 = CFrame.new(-0.5, -1, -0.05)*CFrame.Angles(math.rad(-2),math.rad(0),math.rad(-2.05))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,animationspeed * 1.12505050505,false)

			end
		end


	end
	if animspeed ~= 0 and animmode == "barrage" then
		standanimating = false
		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://5792087636",1,0)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.045,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.5)*CFrame.Angles(math.rad(-15),math.rad(25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 0.5, -1)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0, 0.5)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)


		end

		if animationnumbervalue.Value == 1 then
			createsound("rbxassetid://5792087636",1,0)
			tweenobject3(animationnumbervalue,{Value = 0},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.045,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.5)*CFrame.Angles(math.rad(-15),math.rad(-40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(2, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0.5, -1)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.075,false)

		end

		if animationnumbervalue == 3 then
			animationnumbervalue = 0
		end
	end
	if animspeed ~= 0 and animmode == "punch" then
		standanimating = false
		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://5792087636",.5,0)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.045,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3)*CFrame.Angles(math.rad(10),math.rad(-25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 0.5, 0.5)*CFrame.Angles(math.rad(-30),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0, -0.5)*CFrame.Angles(math.rad(0),math.rad(15),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)


		end
		if animationnumbervalue.Value == 1 then
			tweenobject3(animationnumbervalue,{Value = 1.5},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.5)*CFrame.Angles(math.rad(-15),math.rad(25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 0.5, -1)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0, 0.5)*CFrame.Angles(math.rad(0),math.rad(-25),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)


		end
		if animationnumbervalue.Value == 1.5 then
			animmode = ""
		end

	end
	if animspeed ~= 0 and animmode == "punch2" then
		standanimating = false

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://5792087636",.5,0)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.045,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(-40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3)*CFrame.Angles(math.rad(10),math.rad(40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.25, 0.5, 0.5)*CFrame.Angles(math.rad(-20),math.rad(-40),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-2, 0, 0.5)*CFrame.Angles(math.rad(0),math.rad(-40),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)


		end
		if animationnumbervalue.Value == 1 then
			tweenobject3(animationnumbervalue,{Value = 1.5},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.5)*CFrame.Angles(math.rad(-15),math.rad(-40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(2, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0.5, -1)*CFrame.Angles(math.rad(0),math.rad(40),math.rad(10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.1,false)


		end
		if animationnumbervalue.Value == 1.5 then
			animmode = ""
		end

	end
	if animspeed ~= 0 and animmode == "kick" then
		standanimating = false

		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://5792087636",.5,0)
			tweenobject3(animationnumbervalue,{Value = 1},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.045,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(-0),math.rad(-0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.1)*CFrame.Angles(math.rad(-5),math.rad(-15),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 0, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -2, 0.25)*CFrame.Angles(math.rad(80),math.rad(-180),math.rad(-180))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)


		end
		if animationnumbervalue.Value == 1 then
			tweenobject3(animationnumbervalue,{Value = 1.5},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)	

			tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(-15),math.rad(-15),math.rad(3))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standtorso,{C0 = CFrame.new(0, 0, -3.1)*CFrame.Angles(math.rad(15),math.rad(15),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightarm,{C0 = CFrame.new(1.5, 0, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftarm,{C0 = CFrame.new(-1.5, 0, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -0.65, -0.75)*CFrame.Angles(math.rad(40),math.rad(-10),math.rad(-180))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)
			tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -2, 0)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Circular,0.05,false)


		end
		if animationnumbervalue.Value == 1.5 then
			animmode = ""
		end

	end
	if animspeed ~= 0 and animmode == "" then
		standanimating = false
		animationnumbervalue.Value = 0
		tweenobject2(standhead,{C0 = standheadorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standtorso,{C0 = standtorsoorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standrightarm,{C0 = standrightarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standleftarm,{C0 = standleftarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standrightleg,{C0 = standrightlegorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standleftleg,{C0 = standleftlegorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(rs,{C0 = CFrame.new(1.5, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		tweenobject2(standass1,{C0 = standass1origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standass2,{C0 = standass2origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)

	end
	if animspeed ~= 0 and animmode == "lightdash" then
		animationnumbervalue.Value = 0
		if animationnumbervalue.Value == 0 then
			createsound("rbxassetid://1598682218",.25,0)
			animationnumbervalue.Value = 1
		end

		if standrape.Value == false then
			standanimating = true
		end
		tweenobject2(standhead,{C0 = standheadorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standtorso,{C0 = CFrame.new(0, -2.5, -2)*CFrame.Angles(math.rad(-90),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standrightarm,{C0 = standrightarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standleftarm,{C0 = standleftarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standrightleg,{C0 = standrightlegorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standleftleg,{C0 = standleftlegorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standass1,{C0 = standass1origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		tweenobject2(standass2,{C0 = standass2origin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
		if standrape.Value == false then


			tweenobject2(neck,{C0 = CFrame.new(0, 1, 0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
			tweenobject2(torso,{C0 = CFrame.new(0, -0.093, 0)*CFrame.Angles(math.rad(-20),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
			tweenobject2(rs,{C0 = CFrame.new(1.25, 0.1, -0.45)*CFrame.Angles(math.rad(30),math.rad(10),math.rad(-10))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
			tweenobject2(ls,{C0 = CFrame.new(-1.5, 0.5, 0)*CFrame.Angles(math.rad(-70),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
			tweenobject2(rh,{C0 = CFrame.new(0.5, -0.907, 0)*CFrame.Angles(math.rad(0),math.rad(-10),math.rad(4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
			tweenobject2(lh,{C0 = CFrame.new(-0.5, 0, -1)*CFrame.Angles(math.rad(-10),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.25,false)
		end
	end
	if anim == "idle" then



		if crouching == false then
			for i = 0,2,0.1 do
				tweenobject3(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(0+10*math.cos(sine/70)+LPOR.X),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(torso,{C0 = CFrame.new(0,0+0.1*math.cos(sine/56),0)*CFrame.Angles(math.rad(0),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rh,{C0 = CFrame.new(0.5,-1-0.1*math.cos(sine/56),0)*CFrame.Angles(math.rad(-10+10*math.cos(sine/56)),math.rad(-10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(lh,{C0 = CFrame.new(-0.5,-1-0.1*math.cos(sine/56),0)*CFrame.Angles(math.rad(0),math.rad(10),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)

				tweenobject(ls,{C0 = CFrame.new(-1.5,0.5,0)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rs,{C0 = CFrame.new(1.5,0.5,0)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			end
		end


	end



	if anim == "walk" then
		if holdingshift == false then
			if animmode == "" then
				tweenobject2(standhead,{C0 = CFrame.new(0, 1.60000002, -0.0500000007, 0.819151998, 0, 0.57357651, 0, 1, 0, -0.57357651, 0, 0.819151998)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standtorso,{C0 = CFrame.new(1.5, standtorsoorigin.Position.Y - 1, 2.5, 0.906307757, 0.144543961, -0.397131264, 0, 0.939692616, 0.342020154, 0.42261827, -0.309975505, 0.851650715)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standrightarm,{C0 = standrightarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standleftarm,{C0 = standleftarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standrightleg,{C0 = CFrame.new(0.25, -1.64999998, 0.300000012, -0.974066973, -0.11273405, 0.196174636, 0.0996005237, 0.564862609, 0.819151998, -0.203158051, 0.81744802, -0.53898567)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standleftleg,{C0 = CFrame.new(-0.649999976, -1.70000005, 0.100000001, 0.989228666, -0.0610868037, 0.133022487, -0.133022308, -0.754406512, 0.642787635, 0.0610871762, -0.65355891, -0.754406452)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			end
			for i = 0,2,0.1 do
				tweenobject(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(0-lookvel*3+LPOR.X),math.rad(0-rightvel*10+5*math.cos(sine/120)+tor.RotVelocity.Y*3),math.rad(0+rightvel*4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(torso,{C0 = CFrame.new(0,0+0.3*math.cos(sine/4),0)*CFrame.Angles(math.rad(0-lookvel*5),math.rad(0+5*math.cos(sine/8)),math.rad(0-rightvel*3+tor.RotVelocity.Y*2))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rh,{C0 = CFrame.new(0.5,-1+0.5*math.cos(sine/8),0+0*math.cos(sine/8)*lookvel)*CFrame.Angles(math.rad(0+50*math.sin(sine/8)*lookvel),math.rad(0),math.rad(5+50*math.sin(sine/8))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(lh,{C0 = CFrame.new(-0.5,-1+-0.5*math.cos(sine/8),0+0*math.cos(sine/8)*lookvel)*CFrame.Angles(math.rad(0-50*math.sin(sine/8)*lookvel),math.rad(0),math.rad(-5+-50*math.sin(sine/8))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)

				tweenobject(ls,{C0 = CFrame.new(-1.5,0.5,0+0*math.cos(sine/8)*lookvel)*CFrame.Angles(math.rad(0+50*math.sin(sine/8)*lookvel),math.rad(0),math.rad(5+50*math.sin(sine/8))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rs,{C0 = CFrame.new(1.5,0.5,0+0*math.cos(sine/8)*lookvel)*CFrame.Angles(math.rad(0-50*math.sin(sine/8)*lookvel),math.rad(0),math.rad(-5+-50*math.sin(sine/8))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			end
		end



	end
	if anim == "walk" then
		if holdingshift == true then
			if animmode == "" then
				tweenobject2(standhead,{C0 = CFrame.new(0, 1.55, -0.05)*CFrame.Angles(math.rad(15),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standtorso,{C0 = CFrame.new(2, standtorsoorigin.Position.Y - 1, 1.5)*CFrame.Angles(math.rad(-35),math.rad(0),math.rad(2))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standrightarm,{C0 = standrightarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standleftarm,{C0 = standleftarmorigin},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standrightleg,{C0 = CFrame.new(0.5, -1.65, 0.35)*CFrame.Angles(math.rad(-145),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject2(standleftleg,{C0 = CFrame.new(-0.5, -1.7, 0.1)*CFrame.Angles(math.rad(-130),math.rad(0),math.rad(0))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			end

			for i = 0,2,0.1 do
				tweenobject(neck,{C0 = CFrame.new(0,1,0)*CFrame.Angles(math.rad(10+5*math.cos(sine/2)-lookvel*3+LPOR.X),math.rad(0-rightvel*40+5*math.cos(sine/120)+tor.RotVelocity.Y*3),math.rad(0+rightvel*4))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(torso,{C0 = CFrame.new(0,0+0.3*math.cos(sine/2),0)*CFrame.Angles(math.rad(0-lookvel*20),math.rad(0+5*math.cos(sine/4)),math.rad(0-rightvel*3+tor.RotVelocity.Y*2))},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rh,{C0 = CFrame.new(0.5,-1+0.5*math.cos(sine/4),0+0*math.cos(sine/4)*lookvel)*CFrame.Angles(math.rad(0+100*math.sin(sine/4)*lookvel),math.rad(0),math.rad(5+60*math.sin(sine/4))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(lh,{C0 = CFrame.new(-0.5,-1+-0.5*math.cos(sine/4),0+0*math.cos(sine/4)*lookvel)*CFrame.Angles(math.rad(0-100*math.sin(sine/4)*lookvel),math.rad(0),math.rad(-5+-60*math.sin(sine/4))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)

				tweenobject(ls,{C0 = CFrame.new(-1.5,0.5,0+0*math.cos(sine/4)*lookvel)*CFrame.Angles(math.rad(0+50*math.sin(sine/4)*lookvel),math.rad(0),math.rad(5+50*math.sin(sine/4))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
				tweenobject(rs,{C0 = CFrame.new(1.5,0.5,0+0*math.cos(sine/4)*lookvel)*CFrame.Angles(math.rad(0-50*math.sin(sine/4)*lookvel),math.rad(0),math.rad(-5+-50*math.sin(sine/4))*rightvel-tor.RotVelocity.Y/20)},Enum.EasingDirection.InOut,Enum.EasingStyle.Linear,0.1,false)
			end
		end
	end









	task.wait()



	game.Players.LocalPlayer.Character.Humanoid.BreakJointsOnDeath = false
end
