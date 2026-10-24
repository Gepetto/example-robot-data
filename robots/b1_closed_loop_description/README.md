# Unitree B1 closed-loop model

Full four-leg B1 model with physical split four-bar linkages.  The URDF is a
tree; the four closed loops are described by `loop_constraint` elements in the
SRDF, each constraining x, z, and pitch between the corresponding split-link
end frames.

Source: `/home/b1/unitree_ws/src/unitree_description`, generated from
`robots/b1/b1.urdf.xacro closed_loop:=true`.
