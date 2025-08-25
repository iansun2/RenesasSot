from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from rclpy.duration import Duration
from rclpy.node import Node

def get_audio_note(tone: int, dur: int):
    '''
        tone: 1-7 = C-B \n
        dur : 0-3 = 1.5, 1, 1/2, 1/4
    '''
    FREQ = [523, 587, 659, 698, 783, 880, 987]
    DUR = [1.5, 1, 0.5, 0.25]
    DUR_BASE = 600
    note = AudioNote()
    note.frequency = FREQ[tone-1]
    note.max_runtime = Duration(nanoseconds=DUR_BASE * 1e6 * DUR[dur]).to_msg()
    return note


END_NOTE_LIST = [
    get_audio_note(3, 1),
    get_audio_note(2, 2),
    get_audio_note(1, 1),
    get_audio_note(2, 2),
    get_audio_note(3, 2),
    get_audio_note(4, 3),
    get_audio_note(3, 2),
    get_audio_note(2, 0),
]

READY_NOTE_LIST = [
    get_audio_note(7, 3),
    get_audio_note(7, 3),
]

STEP_NOTE_LIST = [
    get_audio_note(5, 2),
]

LONG_NOTE_LIST = [
    get_audio_note(5, 0),
    get_audio_note(5, 0),
]

START_NOTE_LIST = [
    get_audio_note(1, 1),
    get_audio_note(2, 1),
    get_audio_note(3, 1),
]

# END_NOTE_LIST = [
#     get_audio_note(3, 1),
#     get_audio_note(2, 1),
#     get_audio_note(1, 1),
# ]

class PlatformAudio:
    def __init__(self, node: Node):
        self.node = node
        node.audio_pub = node.create_publisher(
            AudioNoteVector,
            '/cmd_audio',
            2
        )
        pass
    
    def beep_step(self):
        audio_msg = AudioNoteVector()
        audio_msg.notes = STEP_NOTE_LIST
        self.node.audio_pub.publish(audio_msg)

    def beep_long(self):
        audio_msg = AudioNoteVector()
        audio_msg.notes = LONG_NOTE_LIST
        self.node.audio_pub.publish(audio_msg)
    
    def beep_ready(self):
        audio_msg = AudioNoteVector()
        audio_msg.notes = READY_NOTE_LIST
        self.node.audio_pub.publish(audio_msg)
    
    def beep_start(self):
        audio_msg = AudioNoteVector()
        audio_msg.notes = START_NOTE_LIST
        self.node.audio_pub.publish(audio_msg)
    
    def beep_end(self):
        audio_msg = AudioNoteVector()
        audio_msg.notes = END_NOTE_LIST
        self.node.audio_pub.publish(audio_msg)