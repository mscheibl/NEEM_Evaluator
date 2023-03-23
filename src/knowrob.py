import rospy
import rosservice
if 'rosprolog/query' in rosservice.get_service_list():
    from rosprolog_client import Prolog
    prolog = Prolog()
else:
    rospy.logerr("No Knowrob services found")
    raise ImportWarning("No Knowrob services found")
