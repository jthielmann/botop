world {}

### table

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[2. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

Prefix: "L_"
'./panda_moveGripper.g'

Prefix!

Prefix: "R_"
'./panda_moveGripper.g'

Prefix!
        
Edit L_panda_link0 (table) { Q:<t(-.4 -.4 .1) d(90 0 0 1)> }
Edit R_panda_link0 (table)  { Q:<t( .4 -.4 .1) d(90 0 0 1)> }

### camera

camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
