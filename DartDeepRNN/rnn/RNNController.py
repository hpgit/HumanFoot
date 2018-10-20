import tensorflow as  tf
from DartDeepRNN.rnn import Configurations
from DartDeepRNN.util.Pose2d import Pose2d
from copy import copy
from PyCommon.modules.Math import mmMath as mm


class RNNController(object):
    def __init__(self, folder):
        self.config = Configurations.get_config(folder)
        self.config.load_normal_data(folder)
        self.pose = Pose2d()
        
        self.model = self.config.model(1, 1)
        self.sess = tf.Session()
        saver = tf.train.Saver()
        self.sess.run(tf.global_variables_initializer())
        saver.restore(self.sess, "../%s/train/ckpt"%(folder))
        # saver.restore(self.sess, "%s/train/ckpt"%(folder))
        # saver.restore(self.sess, "%s/train/ckpt"%(folder))
        # saver.save(self.sess, "%s/train2/ckpt"%(folder))
        
        self.state = None
        self.current_y = [[0]*self.config.y_normal.size()]

    # def reset(self):
    #     self.state = None
    #     self.current_y = [[0]*self.config.y_normal.size()]

    def step(self, target):
        target = self.config.x_normal.normalize_l(target)
        m = self.model
        feed_dict = {m.x: [[target]], m.prev_y: self.current_y}
        if self.state is not None:
            feed_dict[m.initial_state] = self.state
        
        # x : target x, target y => 2
        # # y : foot contact=2, root transform(rotation, tx, ty)=3, root_height, joint pos=3*13=39  => 45
        # y : foot contact=2, root transform(rotation, tx, ty)=3, root_height, joint pos=3*19=57  => 63
        output, self.state, self.current_y = self.sess.run([m.generated, m.final_state, m.final_y], feed_dict)
        output = output[0][0]
        output = self.config.y_normal.de_normalize_l(output)

        angles = copy(output[63:])
        output = output[:63]
        contacts = copy(output[:2])
        output = output[2:]
        # move root
        self.pose = self.pose.transform(output)
        root_orientation = mm.getSO3FromVectors((self.pose.d[0], 0., self.pose.d[1]), -mm.unitZ())

        points = [[0, output[3], 0]]
        output = output[4:]
        for i in range(len(output)//3):
            points.append(output[i*3:(i+1)*3])

        point_offset = mm.seq2Vec3([0., -0.03, 0.])

        for i in range(len(points)):
            points[i] = mm.seq2Vec3(self.pose.global_point_3d(points[i]))/100. + point_offset

        orientations = list()
        for i in range(len(angles)//3):
            orientations.append(mm.exp(angles[i*3:(i+1)*3]))

        return contacts, points, angles, orientations, root_orientation
