from robosuite.models import MujocoWorldBase
from robosuite.models.robots import SawyerWithGripper
from robosuite.models.robots import Sawyer, Panda
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena, MultiTaskArena
from robosuite.models.objects import BallObject, BreadObject, CerealObject, BoxObject, MugObject
import mujoco
from mujoco import viewer
from moviepy.editor import ImageSequenceClip

def test_sawyer_mt_env():
    world = MujocoWorldBase()
    mujoco_robot = SawyerWithGripper()
    # gripper = gripper_factory('PandaGripper')
    # mujoco_robot.add_gripper(gripper)

    mujoco_robot.set_base_xpos([0, 0, 0])

    world.merge(mujoco_robot)

    mujoco_arena = MultiTaskArena()
    world.merge(mujoco_arena)

    # bread = BreadObject(name="bread")
    # bread_object = bread.get_obj()
    # bread_object.set('pos', '0.0 0.45 0.03')
    # world.worldbody.append(bread_object)
    # world.merge_assets(bread)

    cereal = CerealObject(name="cereal")
    cereal_object = cereal.get_obj()
    cereal_object.set('pos', '0.2 0.45 0.14')
    # cereal_object.set('quat', '0 0 0 1')
    world.worldbody.append(cereal_object)
    world.merge_assets(cereal)

    # box = BoxObject(name="box")
    # box_object = box.get_obj()
    # box_object.set('pos', '-0.2 0.45 0.03')
    # world.worldbody.append(box_object)

    mug = MugObject(name="mug")
    mug_object = mug.get_obj()
    mug_object.set('pos', '-0.2 0.45 0.00')
    world.worldbody.append(mug_object)
    world.merge_assets(mug)

    world.root.find('compiler').set('inertiagrouprange', '0 5')
    world.root.find('compiler').set('inertiafromgeom', 'auto')
    model = world.get_model(mode="mujoco")
    # model = mujoco.MjModel.from_xml_path('/home/saumyas/sims/metaworld/metaworld/envs/assets_v2/Panda_xyz/sawyer_pick_and_place_multitask_gen_onlyblocks_scanned_objs.xml')
    data = mujoco.MjData(model)

    # pos = data.qpos.flat
    # print(len(pos))
    # import ipdb; ipdb.set_trace()
    renderer = mujoco.Renderer(model, 256, 256)
    viewer.launch(model, data)
    imgs = []
    save_gif = True
    for i in range(100):
        mujoco.mj_step(model, data)
        renderer.update_scene(data, camera='left_cap2') 
        img = renderer.render()
        imgs.append(img)

    if save_gif:
        filename = f'./media/test_procedural.gif'
        cl = ImageSequenceClip(imgs, fps=30)
        cl.write_gif(filename, fps=30)

if __name__ == "__main__":
    test_sawyer_mt_env()