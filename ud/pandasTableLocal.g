world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[2. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

obj1(table) {
                Q:[0 0 .075], #(in cm),
                joint:rigid,
                mass:0.2,
          shape:ssBox, size:[.05,.05,.05, 0.], color:[.3 .3 .9],
          contact:1
          }

#L_lift (table){ joint:transZ, limits:[0 .5] }

Prefix: "R_"
Include: '../scenarios/panda_moveGripper.g'
Prefix!
        
Edit R_panda_link0 (table)  { Q:<t( .4 -.4 .1) d(90 0 0 1)> }

camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}

Edit R_finger1{ joint:rigid }
Edit R_finger2{ joint:rigid }
