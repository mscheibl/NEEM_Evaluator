from src.neem_evaluator.knowrob import *
from src.neem_evaluator.events import *
from src.neem_evaluator.neem import *
from src.neem_evaluator.bullet_world import BulletWorld, Object
from src.neem_evaluator.mongo import get_tf_for_object

# Enter the path to your neem here
remember_neem("NEEM")

neem = Neem()

plot_events(neem)

world = BulletWorld()
apartment = Object("apartment", "environment", "apartment.urdf")

tfs = get_tf_for_object("SK_GenesisRightHand")

