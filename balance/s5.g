Include:'./pandasTableLocal.g'

obj1(table) {
	  	Q:[0 0 .075], #(in cm),
	  	joint:rigid,
	  	mass:0.2,
          shape:ssBox, size:[.05,.05,.05, 0.], color:[.3 .3 .9],
          contact:1
          }
        
ref (L_panda_joint6) {shape:marker size:[.2] Q:<t(.07 0 0) d (90 0 1 0)>}
ref3(camera){shape:marker size:[.5]}
