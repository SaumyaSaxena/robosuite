import numpy as np

from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import array_to_string, find_elements, xml_path_completion


class BottleObject(MujocoXMLObject):
    """
    Bottle object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bottle.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class CanObject(MujocoXMLObject):
    """
    Coke can object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/can.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class LemonObject(MujocoXMLObject):
    """
    Lemon object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/lemon.xml"), name=name, obj_type="all", duplicate_collision_geoms=True
        )


class MilkObject(MujocoXMLObject):
    """
    Milk carton object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/milk.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class BreadObject(MujocoXMLObject):
    """
    Bread loaf object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bread.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class CerealObject(MujocoXMLObject):
    """
    Cereal box object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/cereal.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class SquareNutObject(MujocoXMLObject):
    """
    Square nut object (used in NutAssembly)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/square-nut.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle_site"})
        return dic


class RoundNutObject(MujocoXMLObject):
    """
    Round nut (used in NutAssembly)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/round-nut.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle_site"})
        return dic


class MilkVisualObject(MujocoXMLObject):
    """
    Visual fiducial of milk carton (used in PickPlace).

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/milk-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class BreadVisualObject(MujocoXMLObject):
    """
    Visual fiducial of bread loaf (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bread-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class CerealVisualObject(MujocoXMLObject):
    """
    Visual fiducial of cereal box (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/cereal-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class CanVisualObject(MujocoXMLObject):
    """
    Visual fiducial of coke can (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/can-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class PlateWithHoleObject(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/plate-with-hole.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class DoorObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name, friction=None, damping=None, lock=False):
        xml_path = "objects/door.xml"
        if lock:
            xml_path = "objects/door_lock.xml"
        super().__init__(
            xml_path_completion(xml_path), name=name, joints=None, obj_type="all", duplicate_collision_geoms=True
        )

        # Set relevant body names
        self.door_body = self.naming_prefix + "door"
        self.frame_body = self.naming_prefix + "frame"
        self.latch_body = self.naming_prefix + "latch"
        self.hinge_joint = self.naming_prefix + "hinge"

        self.lock = lock
        self.friction = friction
        self.damping = damping
        if self.friction is not None:
            self._set_door_friction(self.friction)
        if self.damping is not None:
            self._set_door_damping(self.damping)

    def _set_door_friction(self, friction):
        """
        Helper function to override the door friction directly in the XML

        Args:
            friction (3-tuple of float): friction parameters to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("frictionloss", array_to_string(np.array([friction])))

    def _set_door_damping(self, damping):
        """
        Helper function to override the door friction directly in the XML

        Args:
            damping (float): damping parameter to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("damping", array_to_string(np.array([damping])))

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle"})
        return dic
    

class RedMugObject(MujocoXMLObject):
    """
    Mug object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/ACE_Coffee_Mug_Kristen_16_oz_cup/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class WhiteMugObject(MujocoXMLObject):
    """
    Mug object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Room_Essentials_Mug_White_Yellow/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class BlueMugObject(MujocoXMLObject):
    """
    Mug object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Cole_Hardware_Mug_Classic_Blue/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class ReebokShoeObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Reebok_TURBO_RC/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class ReebokBlueShoeObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Reebok_CL_RAYEN/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class ReebokBlackShoeObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Reebok_JR_ZIG_COOPERSTOWN_MR/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class ReebokPinkShoeObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Reebok_ZIGLITE_RUSH/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class GreenShoeObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/TROCHILUS_BOOST/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class PinkHeelObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Chelsea_lo_fl_rdheel_zAQrnhlEfw8/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class BrownHeelObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Chelsea_BlkHeelPMP_DwxLtZNxLZZ/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class Supplement0Object(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Hyaluronic_Acid/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class Supplement1Object(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Inositol/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class Supplement2Object(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Folic_Acid/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class CardboardBoxObject(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Google_Cardboard_Original_package/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class BlueLunchPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Olive_Kids_Game_On_Lunch_Box/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class RedLunchPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Paul_Frank_Dot_Lunch_Box/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class WhiteCerealPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class YellowCerealPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Vans_Cereal_Honey_Nut_Crunch_11_oz_box/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class BlueCerealPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Crunch_Girl_Scouts_Candy_Bars_Peanut_Butter_Creme_78_oz_box/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class OrangeCerealPack(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/VANS_FIRE_ROASTED_VEGGIE_CRACKERS_GLUTEN_FREE/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class PurpleLegoSet(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Lego_Friends_Advent_Calendar/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class YellowLegoSet(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/LEGO_Bricks_More_Creative_Suitcase/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class PorcelainPitcher(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Threshold_Porcelain_Pitcher_White/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class PorcelainTeapot(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Threshold_Porcelain_Teapot_White/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class Squirrel(MujocoXMLObject):
    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/mujoco_scanned_objects/Squirrel/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )