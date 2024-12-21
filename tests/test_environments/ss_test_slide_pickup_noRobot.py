from robosuite.models import MujocoWorldBase
from robosuite.models.robots import SawyerWithGripper
from robosuite.models.robots import Sawyer, Panda
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena, MultiTaskNoWallsArena
from robosuite.models.objects import CerealObject
import mujoco
from mujoco import viewer
from moviepy.editor import ImageSequenceClip
from robosuite.models.objects.primitive.box import BoxObject


# import os
# NVIDIA_ICD_CONFIG_PATH = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
# if not os.path.exists(NVIDIA_ICD_CONFIG_PATH):
#   with open(NVIDIA_ICD_CONFIG_PATH, 'w') as f:
#     f.write("""{
#     "file_format_version" : "1.0.0",
#     "ICD" : {
#         "library_path" : "libEGL_nvidia.so.0"
#     }
# }
# """)

# # Configure MuJoCo to use the EGL rendering backend (requires GPU)
# print('Setting environment variable to use GPU rendering:')
# os.environ['MUJOCO_GL']='egl'

def test_sawyer_mt_env():
    world = MujocoWorldBase()
    # mujoco_robot = SawyerWithGripper()

    # mujoco_robot.set_base_xpos([0, 0, 0])

    # world.merge(mujoco_robot)

    mujoco_arena = MultiTaskNoWallsArena()
    world.merge(mujoco_arena)

    block_bottom = BoxObject(name='Block_bottom', size=[0.1, 0.1, 0.03], rgba=[0, 1, 0, 1])
    block_top = BoxObject(name='Block_top', size=[0.1, 0.1, 0.03], rgba=[1, 0, 0, 1])

    block_bottom_body = block_bottom.get_obj()
    block_bottom_body.set('pos', f'0. 0.6 {-block_bottom.bottom_offset[2]}')
    # block_bottom_body.set('pos', f'0.6 0.6 0.5')
    world.worldbody.append(block_bottom_body)
    world.merge_assets(block_bottom)

    block_top_z = (block_bottom.top_offset - block_bottom.bottom_offset - block_top.bottom_offset)[2]
    block_top_body = block_top.get_obj()
    block_top_body.set('pos', f'0. 0.6 {block_top_z}')
    # block_top_body.set('pos', f'0.3 0.45 {-block_top.bottom_offset[2]}')
    world.worldbody.append(block_top_body)
    world.merge_assets(block_top)
    
    # add distractors
    block_dist1 = BoxObject(name='Block_dist1', size=[0.03, 0.03, 0.03], rgba=[1, 1, 0, 1])
    block_dist1_body = block_dist1.get_obj()
    block_dist1_body.set('pos', f'0. 0.3 {-block_dist1.bottom_offset[2]}')
    world.worldbody.append(block_dist1_body)
    world.merge_assets(block_dist1)

    block_dist2 = BoxObject(name='Block_dist2', size=[0.03, 0.03, 0.03], rgba=[1, 0, 1, 1])
    block_dist2_body = block_dist2.get_obj()
    block_dist2_body.set('pos', f'0.4 0.6 {-block_dist2.bottom_offset[2]}')
    world.worldbody.append(block_dist2_body)
    world.merge_assets(block_dist2)

    world.root.find('compiler').set('inertiagrouprange', '0 5')
    world.root.find('compiler').set('inertiafromgeom', 'auto')
    model = world.get_model(mode="mujoco")
    data = mujoco.MjData(model)
    
    renderer = mujoco.Renderer(model, 480, 480)
    # viewer.launch(model, data)
    imgs = []
    save_gif = True

    # test force applied
    if True:
        data.xfrc_applied[model.body('Block_bottom_main').id] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(20):
            mujoco.mj_step(model, data)
            renderer.update_scene(data, camera='left_cap3') 

            img = renderer.render()
            imgs.append(img)

        if save_gif:
            filename = f'./media/test_safety_env_dist.gif'
            cl = ImageSequenceClip(imgs[::4], fps=500)
            cl.write_gif(filename, fps=500)
    
    # test reset
    if False:
        print(data.xpos[model.body('Block_bottom_main').id])
        mujoco.mj_step(model, data)
        print(data.xpos[model.body('Block_bottom_main').id])
        # data.xpos[model.body('Block_bottom_main').id] = [0.0, 0.5, 0.03]
        data.qpos[:3] = [0.0, 0.5, 0.03]
        # print(data.xpos[model.body('Block_bottom_main').id])
        print(data.qpos[:3])
        mujoco.mj_forward(model, data)
        print(data.xpos[model.body('Block_bottom_main').id])
        import ipdb; ipdb.set_trace()
        mujoco.mj_resetData(model, data)
        
        print(data.xpos[model.body('Block_bottom_main').id])
        import ipdb; ipdb.set_trace()


if __name__ == "__main__":
    test_sawyer_mt_env()