from .knowrob import get_all_actions_in_neem, get_all_tf_for_action, map_sequence_to_action
from .bullet_world import BulletWorld, Object
from .metrics import vel_metric
from .mongo import get_tf_for_object
from .helper import cluster_sequences, cluster_to_actions


def event_vis():
    world = BulletWorld()
    apartment = Object("apartment", "env", "apartment.urdf")
    hand_ont = 'http://knowrob.org/kb/iai-apartment.owl#right_hand_1'
    event_colors = {"Reaching": [1, 0, 0],
                    "Grasping": [1, 1, 0],
                    "Pouring": [0, 1, 1]}

    actions = get_all_actions_in_neem()
    for act, readable in actions.items():
        if 'Physical' in readable:
            continue
        tfs = get_all_tf_for_action(act)
        hand_tfs = tfs[hand_ont]
        for action_type in event_colors.keys():
            if action_type in readable:
                world.visualize_trajectory(hand_tfs, color=event_colors[action_type])


def metric_and_cluster(child_frame_id):
    tfs = get_tf_for_object(child_frame_id)
    metric = vel_metric(tfs, 1, True)
    cluster = cluster_sequences(metric[1])
    seq_to_action = map_sequence_to_action('http://knowrob.org/kb/iai-apartment.owl#right_hand_1')
    cls_t_act = cluster_to_actions(cluster, seq_to_action)

