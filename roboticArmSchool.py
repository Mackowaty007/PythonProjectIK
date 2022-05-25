import tkinter as tk
import math
import serial
import time

#declare variables
HEIGHT = 800
WIDTH = 800

rotSpeed = 0.01
pointSpeed = 0.1
grabSpeed = 1.0
armLength1 = 4.9*20
armLength2 = 4.9*20
armLength3 = 6.4*20
floorLevel = -3.7*20

minServoAngle = [-math.pi/2,  math.pi,-math.pi/2,-math.pi/2]
maxServoAngle = [ math.pi/2,2*math.pi, math.pi/2, math.pi/2]
maxGrab = 20
minGrab = 0

baseRot = 0
trueBaseRot = 0
arm1rot = 0
true1rot = 0
arm2rot = 0
true2rot = 0
arm3rot = 0
true3rot = 0
grab = maxGrab
goToPoint = [1,0]
child4swingPos = [1,0]
rotationOfChild4 = math.pi

#gets the angle between two lines
def computeTheAngle(pointX1,pointY1,pointX2,pointY2):
	x = pointX2-pointX1
	y = pointY2-pointY1
	
	#hacky solution - cringe edition
	if x == 0:
		x=0.1
	
	alphaAngle = math.atan(y/x)
	if (pointX2 < pointX1):
		alphaAngle = math.atan(y/x) + math.pi
	return alphaAngle

#finds intersection of two wheels
def intersect(radius1,radius2,x1,y1,x2,y2):
	d = math.sqrt(pow(x2 - x1,2)+pow(y2 - y1,2))
	#bieda edition
	if d == 0:
		d = 0.01
	ex = (x2 - x1) / d
	ey = (y2 - y1) / d
	x = (radius1 * radius1 - radius2 * radius2 + d * d) / (2 * d)
	y = math.sqrt(radius1 * radius1 - x * x)
	#P1 = [x1 + x * ex - y * ey  , y1 + x * ey + y * ex]
	P2 = [x1 + x * ex + y * ey,  y1 + x * ey - y * ex]
	return P2

#tkinter init
root = tk.Tk()
root.title("robotic arm")
root.geometry("800x800")
canvas=tk.Canvas(root,width=WIDTH,height=HEIGHT)
canvas.pack()

#serial comunications init
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

def onKeyPress(event):
	global rotationOfChild4
	global baseRot
	global grab
	if event.char == 'q':
		rotationOfChild4 = rotationOfChild4 - math.pi/40
	if event.char == 'e':
		rotationOfChild4 = rotationOfChild4 + math.pi/40
	if event.char == 'a':
		baseRot = baseRot - math.pi/40
	if event.char == 'd':
		baseRot = baseRot + math.pi/40
	if event.char == 'w':
		grab = grab + grabSpeed
	if event.char == 's':
		grab = grab - grabSpeed
	#limit the angle of servo1
	baseRot = max(min(baseRot,maxServoAngle[0]),minServoAngle[0])
	#limit the grab
	grab = max(min(grab,maxGrab),minGrab)

def main():
	global baseRot
	global arm1rot
	global arm2rot
	global arm3rot
	global rotSpeed
	global pointSpeed
	global grabSpeed
	global armLength1 
	global armLength2
	global armLength3
	global floorLevel
	global maxServoAngle
	global minServoAngle
	global goToPoint
	global child4swingPos
	global rotationOfChild4

	#clear the screen
	canvas.delete("all")

	#mouse input
	x = root.winfo_pointerx()
	y = root.winfo_pointery()
	goToPoint[0] = -(root.winfo_pointerx() - root.winfo_rootx() - WIDTH/2)
	goToPoint[1] = -(root.winfo_pointery() - root.winfo_rooty() - HEIGHT/2)

	#don't let the arm go beneth the floor
	if goToPoint[1] < floorLevel:
		goToPoint[1] = floorLevel

	arm3rot = math.pi -(arm1rot + arm2rot) + rotationOfChild4

	child4swingPos = [goToPoint[0] - math.cos(rotationOfChild4)*armLength3,goToPoint[1] - math.sin(rotationOfChild4)*armLength3]

	#don't let the second arm go beneth the floor
	if child4swingPos[1] < floorLevel:
		child4swingPos[1] = floorLevel

	#calculate the arm angles
	if armLength1 + armLength2 > math.sqrt(math.pow(child4swingPos[0],2) + math.pow(child4swingPos[1],2)):
		intersectionPoint = intersect(armLength1,armLength2,0,0,child4swingPos[0],child4swingPos[1])
		arm1rot = math.pi + computeTheAngle(0,0,intersectionPoint[0],intersectionPoint[1])
		arm2rot = math.pi + -arm1rot + computeTheAngle(intersectionPoint[0],intersectionPoint[1],child4swingPos[0],child4swingPos[1])
	#elif armLength1 + armLength2 + armLength3 > math.sqrt(math.pow(goToPoint[0],2)+math.pow(goToPoint[1],2)):
		#this i might add later cuz it's  annoying as hell but i got no time to implement it
	else:
		#fix a cringy division by zero bug
		if goToPoint[0]==0:
			goToPoint[0] = 0.01
		if goToPoint[1]==0:
			goToPoint[1] = 0.01

		if goToPoint[0] < 0:
			arm1rot = math.atan(goToPoint[1]/goToPoint[0]) + math.pi*2
		else:
			arm1rot = math.atan(goToPoint[1]/goToPoint[0]) + math.pi
		arm2rot = 0
		arm3rot = 0

    
	arm1rot = max(min(arm1rot,maxServoAngle[1]),minServoAngle[1])
	#this fixes a funny bug
	#if arm2rot < -math.pi:
	#	arm2rot = arm2rot + math.pi*2
	arm2rot = max(min(arm2rot,maxServoAngle[2]),minServoAngle[2])
	arm3rot = max(min(arm3rot,maxServoAngle[3]),minServoAngle[3])
	
	trueBaseRot = int(baseRot/math.pi*180)+90
	true1rot = int((arm1rot/math.pi)*180)-180
	true2rot = int((arm2rot/math.pi)*180)+90
	true3rot = int((arm3rot/math.pi)*180)+90
	#print("1: " + str(true1rot) + "2: " + str(true2rot) + "3: " + str(true3rot))
	
	#serial communication stuff
	ser.write(bytes(str(true1rot) + "\n",'utf-8'))
	ser.write(bytes(str(true2rot) + "\n",'utf-8'))
	ser.write(bytes(str(true3rot) + "\n",'utf-8'))
	ser.write(bytes(str(trueBaseRot) + "\n",'utf-8'))
	line = ser.readline().decode('utf-8').rstrip()
	#print (line)

	#draw
	#the floor
	canvas.create_rectangle(0, HEIGHT/2-floorLevel, WIDTH, HEIGHT, fill='#00300a')
	#debug cube
	canvas.create_line(-goToPoint[0]+WIDTH/2,-goToPoint[1]+HEIGHT/2,-goToPoint[0]+WIDTH/2+math.cos(rotationOfChild4)*20,-goToPoint[1]+HEIGHT/2+math.sin(rotationOfChild4)*20,fill="red",width=5)
	canvas.create_rectangle(-child4swingPos[0]+WIDTH/2-5,-child4swingPos[1]+HEIGHT/2-5,-child4swingPos[0]+WIDTH/2+5,-child4swingPos[1]+HEIGHT/2+5,fill='#12345a')
	#arms
	canvas.create_line(
		WIDTH/2,
		HEIGHT/2,
		WIDTH/2+math.cos(arm1rot)*armLength1,
		HEIGHT/2+math.sin(arm1rot)*armLength1,
		fill="blue",width=5)
	canvas.create_line(
		WIDTH/2+math.cos(arm1rot)*armLength1,
		HEIGHT/2+math.sin(arm1rot)*armLength1,
		WIDTH/2+math.cos(arm1rot)*armLength1+math.cos(arm1rot+arm2rot)*armLength2,
		HEIGHT/2+math.sin(arm1rot)*armLength1+math.sin(arm1rot+arm2rot)*armLength2,
		fill="blue",width=5)
	canvas.create_line(
		WIDTH/2+math.cos(arm1rot)*armLength1+math.cos(arm1rot+arm2rot)*armLength2,
		HEIGHT/2+math.sin(arm1rot)*armLength1+math.sin(arm1rot+arm2rot)*armLength2,
		WIDTH/2+math.cos(arm1rot)*armLength1+math.cos(arm1rot+arm2rot)*armLength2+math.cos(arm1rot+arm2rot+arm3rot)*armLength3,
		HEIGHT/2+math.sin(arm1rot)*armLength1+math.sin(arm1rot+arm2rot)*armLength2+math.sin(arm1rot+arm2rot+arm3rot)*armLength3,
		fill="blue",width=5)
	#show rotation
	canvas.create_line(
		100,
		100,
		100+math.cos(baseRot-math.pi/2)*70,
		100+math.sin(baseRot-math.pi/2)*70,
		fill="blue",width=5)
	canvas.create_line(
		100+math.cos(baseRot-math.pi/2)*70+math.cos(baseRot-math.pi/2+math.pi/2)*grab,
		100+math.sin(baseRot-math.pi/2)*70+math.sin(baseRot-math.pi/2+math.pi/2)*grab,
		100+math.cos(baseRot-math.pi/2)*70+math.cos(baseRot-math.pi/2+math.pi/2)*grab+math.cos(baseRot-math.pi/2)*20,
		100+math.sin(baseRot-math.pi/2)*70+math.sin(baseRot-math.pi/2+math.pi/2)*grab+math.sin(baseRot-math.pi/2)*20,
		fill="blue",width=5)
	canvas.create_line(
		100+math.cos(baseRot-math.pi/2)*70-math.cos(baseRot-math.pi/2+math.pi/2)*grab,
		100+math.sin(baseRot-math.pi/2)*70-math.sin(baseRot-math.pi/2+math.pi/2)*grab,
		100+math.cos(baseRot-math.pi/2)*70-math.cos(baseRot-math.pi/2+math.pi/2)*grab+math.cos(baseRot-math.pi/2)*20,
		100+math.sin(baseRot-math.pi/2)*70-math.sin(baseRot-math.pi/2+math.pi/2)*grab+math.sin(baseRot-math.pi/2)*20,
		fill="blue",width=5)

	root.after(2,main)

root.bind('<KeyPress>', onKeyPress)
root.after(2,main)
root.mainloop()