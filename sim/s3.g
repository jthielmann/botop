Include:'../../scenarios/pandasTable.g'

obj1(table) {
	  	Q:[0 0 .075], #(in cm),
	  	joint:rigid,
	  	mass:0.2,
          shape:ssBox, size:[.5,.05,.07, 0.], color:[.3 .3 .9],
          contact:1
          }

stick(table) {
		Q:[0.5 0.5 .075], #(in cm),
		joint:rigid,
		#mass:0.2,
        shape:ssBox, size:[.01,.01,.05, 0.], color:[.3 .3 .9]
        }
        
ball(stick) {
		Q:[0 0 0.04], #(in cm),
        shape:sphere, size:[0.03], color:[.3 .3 .9],
        
        #shape:ssBox, size:[.03,.03,.03, 0.], color:[.3 .3 .9]
        }

#nix(ball) {shape:marker size:[.5]}
          
#obj2(obj1) {
#  Q:[0 0 .1] #(in cm)
#  joint:rigid
#shape:ssBox, size:[.1,.1,.1, 0.], color:[.3 .3 .9]}
