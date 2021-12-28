Include:'../../scenarios/pandasTable.g'

obj1(table) {
	  	Q:[0 0 .075], #(in cm),
	  	joint:rigid,
	  	mass:0.2,
	  	friction:.5,
          shape:ssBox, size:[.05,.05,.05, 0.], color:[.3 .3 .9],
          contact:1
          }
          
obj2(obj1) {
	  	Q:[0 0 .05], #(in cm),
	  	joint:rigid,
	  	mass:0.2,
	  	friction:.5,
          shape:ssBox, size:[.05,.05,.05, 0.], color:[.3 .3 .9],
          contact:1
          }

        
#nix(ball) {shape:marker size:[.5]}
          
#obj2(obj1) {
#  Q:[0 0 .1] #(in cm)
#  joint:rigid
#shape:ssBox, size:[.1,.1,.1, 0.], color:[.3 .3 .9]}
