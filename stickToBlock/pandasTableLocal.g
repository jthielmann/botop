world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[2. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

obj1(table) {
          Q:[-0.05 -0.10 .065], #(in cm),
	  joint:rigid,
	  mass:0.2,
          shape:ssBox, size:[.5,.03,.03, 0.01], color:[.3 .3 .9],
          contact:1
          }


ball(table) {
                Q:[-.1 -.3 .075], #(in cm),
                joint:rigid,
                mass:0.2,
          shape:ssBox, size:[.07,.07,.07, 0.], color:[.3 .3 .9],
          contact:1
          }

#L_lift (table){ joint:transZ, limits:[0 .5] }

Prefix: "l_"
Include: '../scenarios/panda_moveGripper.g'
Prefix!

Edit l_panda_link0 (table)  { Q:<t( .4 -.4 .05) d(90 0 0 1)> }

camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
ref (obj1) {shape:marker size:[.2] Q:<t(.07 0 0) d (0 0 1 0)>}
ref2 (ball) {shape:marker size:[.2] Q:<t(.07 0 0) d (0 0 1 0)>}
refTable(ball) {shape:marker size:[.2] Q:<t(-.3 0 -.15)>}

Edit l_finger1{ joint:rigid }
Edit l_finger2{ joint:rigid }


stickTop (l_gripperCenter) {shape:marker size:[.2] Q:<d(45 1 0 0) t(0 0 -0.5)>}

