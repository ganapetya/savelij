"""No motion.

Masha publishes action files to a hexapod. Savelij is a Rosmaster. This
module does not publish, and it does not write servo pulses.
"""


def jaw_while(still_playing, publish, complete, name: str) -> None:
    del still_playing, publish, complete, name


def play_once(publish, complete, name: str) -> None:
    del publish, complete, name
