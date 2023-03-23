import pymongo
import os
import bson

client = pymongo.MongoClient('mongodb://localhost:27017/')
db = client["roslog"]
tf = db["tf"]

def get_object_names():
    return tf.distinct("child_frame_id")

def get_tf_for_object(name):
    return tf.find({"child_frame_id": name})

def restore(path):
    """
    MongoDB Restore
    :param path: Database dumped path
    :param conn: MongoDB client connection
    :param db_name: Database name
    :return:

    >>> DB_BACKUP_DIR = '/path/backups/'
    >>> conn = MongoClient("mongodb://admin:admin@127.0.0.1:27017", authSource="admin")
    >>> db_name = 'my_db'
    >>> restore(DB_BACKUP_DIR, conn, db_name)

    """

    #db = conn[db_name]
    if set(['triples', 'tf', 'annotations']) <= set(os.listdir(path)):
        coll_paths = [path + "/" + x + "/roslog/" for x in ['triples', 'tf', 'annotations']]
    else:
        print("The given path is no valid NEEM")
        return
    for coll in coll_paths:
        for file in os.listdir(coll):
            if file.endswith('.bson'):
                with open(os.path.join(coll, file), 'rb+') as f:
                    db[file.split('.')[0]].insert_many(bson.decode_all(f.read()))
    print("Successfully loaded NEEM")

def remove_neem():
    return client.drop_database("roslog")
