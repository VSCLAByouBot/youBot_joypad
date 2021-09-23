keyboard_msg = """
Reading from the keyboard and Publishing to youBot!
------------------------------------------------------
Arm Control:                 Base Moving:

+/- x coord : w/x             ↖      ↑      ↗
+/- y coord : a/d               u    i    o
+/- z coord : q/z            ←  j         l →
                                m    ,    .
                              ↙	     ↓      ↘

t/y : increase/decrease max speeds of the base
b/n : increase/decrease max speeds of the arm
g   : show current joint state
h   : show current end-effector position
anything else : stop

CTRL-C to quit
"""

baseBindings = {
#	x,y,tetha ratio
	'i':( 1, 0, 0), 	# forwards
	'o':( 1, 0,-1), 	# forwards + rotation right
	'j':( 0, 1, 0), 	# left
	'l':( 0,-1, 0),		# right
	'u':( 1, 0, 1), 	# forwards + rotation left
	',':(-1, 0, 0), 	# backward
	'.':( 0, 0,-1), 	# turn right on spot
	'm':( 0, 0, 1), 	# turn left  on spot
	#'k':( 0, 0, 0),
}
ratioBindings = {
# 	base, arm speed ratio
	't': ( 0.1,   0),
	'y': (-0.1,   0),
	'b': (   0, 0.1),
	'n': (   0,-0.1),
	#'k': (   0,   0),  
}
armBindings = {
#	x, y, z 
	'w': ( 1, 0, 0),
	'x': (-1, 0, 0),
	'a': ( 0, 1, 0),
	'd': ( 0,-1, 0),
	'q': ( 0, 0, 1),
	'z': ( 0, 0,-1),
	#'k': ( 0, 0, 0),
}
jointBindings = {
#	J1 ~ J5
	'r': ( 1, 0, 0, 0, 0),
	't': (-1, 0, 0, 0, 0),
}

stopBindings = {
	"base" : (0, 0, 0),
	"arm"  : (0, 0, 0),
	"joint": (0, 0, 0, 0, 0),
	"ratio": (0, 0),
}
