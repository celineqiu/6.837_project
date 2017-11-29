# 6.837_project

HE said the paper we were looking at is p shitty

Easier to start with below steps instead:

-create a secondary skin mesh
-attach secondary skin mesh to non-moving skin mesh with springs
    -what's a good way to calculate spring constant?
    -justin suggested looking at the number of bones a point on the skin is attached to
      i.e. a piece of skin is more "jiggly" if it's only attached to one bone, vs many -- yes?
      -in this case, maybe use something proportional to squared sum of weights 
        we can also test out other math
        
-use integrator to calculate motion of secondary skin mesh (the secondary mesh is the displayed mesh)

To add/combine from the previous assn2/3:
  -secondary skin mesh
  -vertex velocities
  -springs between the two meshes
  -time stepper
  -integrator
  
To do for check in 11/29:
  -get the skeleton to move
  -maybe have some different movements, i.e.
     -jumping
     -walking
     
  -then write out what we plan to do, i.e. the spring stuff, math behind the spring stuff
  
  
  If we want to do more difficult stuff:
  https://graphics.pixar.com/library/WigglySplinesA/paper.pdf
