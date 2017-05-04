from flask import Flask, request, jsonify
from paraphrase_detector import Paraphrase_detector
import skipthoughts

# use the skipthought model to create an encoder which 
# must be passed to the paraphrase detector.
# Creating it here is more efficient than in the paraphrase
# detector itself because the paraphrase detector can 
# then be discarded and a new one can be created 
# without having to reload the weights of the 
# skipthought model.
model = skipthoughts.load_model()
encoder = skipthoughts.Encoder(model)

# create the paraphraser
paraphraser = Paraphrase_detector(encoder)

app = Flask(__name__)


# use this to detect paraphrases
@app.route('/paraphrase_this', methods=['GET', 'POST'])
def return_closest_sentence():
    # import IPython
    # IPython.embed()
    sentence = request.json['data']
    closest_sentence = paraphraser.check_task_list_for_paraphrases(sentence)
    return jsonify({"data": closest_sentence})


# use this to add a pose task
@app.route('/add_pose_task', methods=['GET', 'POST'])
def add_pose_task():
    pose_string = request.json['data']
    paraphraser.add_sentence('pose', pose_string)
    return "ok"


# use this to add an object task (e.g. grasp an object)
@app.route('/add_object_task', methods=['GET', 'POST'])
def add_object_task():
    object_string = request.json['data']
    paraphraser.add_sentence('object', object_string)
    return "ok"


# use this to add a place task (e.g. go to a place)
@app.route('/add_place_task', methods=['GET', 'POST'])
def add_place_task():
    place_string = request.json['data']
    paraphraser.add_sentence('place', place_string)
    return "ok"


# use this to add poses, objects, and places to commands
# known to the paraphraser. Useful for testing 
# Modify the underlying paraphraser method to change
# objects, poses, and places added.
@app.route('/start_test_env', methods=['GET', 'POST'])
def start_test_env():
    paraphraser.instantiate_test_env()
    return "ok"


# Add an entirely new command to the list known to the paraphraser
# Do not use for object, pose, or place commands
@app.route('/add_new_command', methods=['GET', 'POST'])
def add_new_command():
    new_command = request.json['data']
    paraphraser.add_command(new_command)
    return "ok"


# Reset the paraphraser so that it forgets all the poses, objects,
# places, and new commands that have been given to it
@app.route('/reset', methods=['GET', 'POST'])
def reset():
    paraphraser = Paraphrase_detector(encoder)
    return "ok"


if __name__ == '__main__':
    app.run()
