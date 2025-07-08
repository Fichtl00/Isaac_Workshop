import omni.usd 
import omni.kit.commands
from pxr import Gf 
# Get current stage 
stage = omni.usd.get_context().get_stage() 
# Create cube with specified parameters 
result, prim_path = omni.kit.commands.execute( 
	"CreateMeshPrim",
	prim_type="Cube",
	prim_path="/World/Cube",
	object_origin=Gf.Vec3f(0, 0, 0),
	half_scale=50.0,
	u_verts_scale=1,
	v_verts_scale=1,
	w_verts_scale=1 )

