from robosuite.models.objects.primitive.box import *
from robosuite.models.objects.xml_objects import *

def get_obj_from_name(obj_name):
    if 'white_mug' in obj_name:
        model = WhiteMugObject(name=obj_name)
    elif 'bread' in obj_name:
        model = BreadObject(name=obj_name)
    elif 'red_mug' in obj_name:
        model = RedMugObject(name=obj_name)
    elif 'blue_mug' in obj_name:
        model = BlueMugObject(name=obj_name)
    elif 'box' in obj_name or 'block' in obj_name:
        model = BoxObject(name=obj_name, size=[0.024, 0.024, 0.03])
    elif 'red_cereal' in obj_name:
        model = CerealObject(name=obj_name)
    elif 'reebok_shoe' in obj_name:
        model = ReebokShoeObject(name=obj_name)
    elif 'reebok_blue_shoe' in obj_name:
        model = ReebokBlueShoeObject(name=obj_name)
    elif 'reebok_black_shoe' in obj_name:
        model = ReebokBlackShoeObject(name=obj_name)
    elif 'reebok_pink_shoe' in obj_name:
        model = ReebokPinkShoeObject(name=obj_name)
    elif 'green_shoe' in obj_name:
        model = GreenShoeObject(name=obj_name)
    elif 'pink_heel' in obj_name:
        model = PinkHeelObject(name=obj_name)
    elif 'brown_heel' in obj_name:
        model = BrownHeelObject(name=obj_name)
    elif 'supplement0' in obj_name:
        model = Supplement0Object(name=obj_name)
    elif 'supplement1' in obj_name:
        model = Supplement1Object(name=obj_name)
    elif 'supplement2' in obj_name:
        model = Supplement2Object(name=obj_name)
    elif 'cardboard' in obj_name:
        model = CardboardBoxObject(name=obj_name)
    elif 'blue_lunch_pack' in obj_name:
        model = BlueLunchPack(name=obj_name)
    elif 'red_lunch_pack' in obj_name:
        model = RedLunchPack(name=obj_name)
    elif 'white_cereal_pack' in obj_name:
        model = WhiteCerealPack(name=obj_name)
    elif 'yellow_cereal_pack' in obj_name:
        model = YellowCerealPack(name=obj_name)
    elif 'blue_cereal_pack' in obj_name:
        model = BlueCerealPack(name=obj_name)
    elif 'orange_cereal_pack' in obj_name:
        model = OrangeCerealPack(name=obj_name)
    elif 'purple_lego_set' in obj_name:
        model = PurpleLegoSet(name=obj_name)
    elif 'yellow_lego_set' in obj_name:
        model = YellowLegoSet(name=obj_name)
    elif 'porcelain_pitcher' in obj_name:
        model = PorcelainPitcher(name=obj_name)
    elif 'porcelain_teapot' in obj_name:
        model = PorcelainTeapot(name=obj_name)
    elif 'porcelain_mug' in obj_name:
        model = PorcelainMug(name=obj_name)
    elif 'squirrel' in obj_name:
        model = Squirrel(name=obj_name)
    else:
        raise NotImplementedError(f'Object with name {obj_name} not available.')
    return model