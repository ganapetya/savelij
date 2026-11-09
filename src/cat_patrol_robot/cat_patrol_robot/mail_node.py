#!/usr/bin/env python3
# =============================================================================
# mail_node.py — ROS 2 Python node that sends patrol photos via email
# =============================================================================
#
# HOW IT WORKS:
#   1. Subscribes to a ROS topic (/cat_patrol/mail_request).
#   2. Receives a JSON string with {subject, to, paths[]}.
#   3. Builds an email with those JPEG files as attachments.
#   4. Sends via SMTP (Gmail, Outlook, etc.) using environment variables.
#
# WHY A SEPARATE NODE?
#   Email sending requires SMTP libraries, credentials, and network access.
#   Keeping this in a separate Python node means:
#   - The C++ patrol_node stays simple (no SMTP complexity).
#   - Credentials are isolated (only this node needs them).
#   - You can restart/update the mail logic without touching the patrol code.
#   - Python's smtplib and email modules make this trivial.
#
# ROS 2 CONCEPT: PYTHON NODES vs C++ NODES
#   ROS 2 supports both Python (rclpy) and C++ (rclcpp) nodes.  They
#   communicate via the SAME topics, services, and actions — the language
#   doesn't matter.  Use C++ for performance-critical code (sensor processing,
#   motor control) and Python for convenience (email, web APIs, scripting).
#
# SECURITY:
#   SMTP credentials come from ENVIRONMENT VARIABLES, not config files.
#   This avoids accidentally committing passwords to git.
#   Set them in your shell profile, systemd service, or a .env file:
#     export CAT_PATROL_SMTP_HOST=smtp.gmail.com
#     export CAT_PATROL_SMTP_PORT=587
#     export CAT_PATROL_SMTP_USER=you@gmail.com
#     export CAT_PATROL_SMTP_PASSWORD=your_app_password
#     export CAT_PATROL_SMTP_FROM=you@gmail.com  (optional, defaults to USER)
#
# MESSAGE FORMAT (JSON on std_msgs/String):
#   {"subject": "Cat patrol photos",
#    "to": "user@example.com",
#    "paths": ["/tmp/cat_patrol_images/snap_12345_0.jpg", ...]}
# =============================================================================

import json            # Parse JSON strings into Python dicts
import mimetypes       # Guess file MIME type from extension (e.g., .jpg → image/jpeg)
import os              # Read environment variables, check file existence
import smtplib         # Connect to SMTP servers and send emails
from email.message import EmailMessage  # Build structured email messages

# ---------------------------------------------------------------------------
# ROS 2 Python imports
# ---------------------------------------------------------------------------
# rclpy        — the Python equivalent of rclcpp (C++ ROS 2 library)
# rclpy.node   — base class for Python nodes (like rclcpp::Node in C++)
# std_msgs.msg — standard message types (String, Bool, Int32, etc.)
#
# PYTHON ROS 2 vs C++ ROS 2 — KEY DIFFERENCES:
#   C++:    class MyNode : public rclcpp::Node { ... };
#   Python: class MyNode(Node): ...
#
#   C++:    rclcpp::init(argc, argv);  auto node = std::make_shared<MyNode>();
#   Python: rclpy.init();              node = MyNode()
#
#   C++:    rclcpp::spin(node);
#   Python: rclpy.spin(node)
#
#   C++:    declare_parameter<std::string>("name", "default");
#   Python: self.declare_parameter('name', 'default')
#
#   C++:    create_subscription<String>(topic, qos, callback);
#   Python: self.create_subscription(String, topic, callback, qos)
#           Note: in Python, QoS (queue depth) is the LAST argument,
#           while in C++ it comes before the callback.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MailNode(Node):
    """ROS 2 node that listens for mail requests and sends emails via SMTP."""

    def __init__(self):
        # PYTHON ROS 2 CONCEPT: super().__init__('node_name')
        #   Every Python node must call Node.__init__ with a unique name.
        #   This name appears in `ros2 node list` and log output.
        super().__init__('cat_patrol_mail')

        # ROS 2 CONCEPT: PARAMETERS IN PYTHON
        #   declare_parameter(name, default_value) registers a parameter.
        #   get_parameter(name) retrieves its current value.
        #   Parameters can be set via YAML, launch file, or command line:
        #     ros2 run cat_patrol_robot mail_node --ros-args -p mail_request_topic:=/my_topic
        self.declare_parameter('mail_request_topic', '/cat_patrol/mail_request')
        topic = self.get_parameter('mail_request_topic').get_parameter_value().string_value

        # ROS 2 CONCEPT: SUBSCRIPTION IN PYTHON
        #   create_subscription(MsgType, topic, callback, queue_depth)
        #   Every time a message is published on `topic`, `self._cb` is called
        #   with the message as its argument.
        #
        #   Queue depth = 10: buffer up to 10 unprocessed messages.  If the
        #   callback is slow (network I/O for SMTP), older messages may be dropped.
        self.sub = self.create_subscription(String, topic, self._cb, 10)

    def _cb(self, msg: String):
        """Called whenever a mail request message arrives."""

        # PYTHON CONCEPT: JSON PARSING
        #   json.loads() converts a JSON string into a Python dict:
        #     '{"a": 1, "b": [2, 3]}' → {'a': 1, 'b': [2, 3]}
        #   If the string isn't valid JSON, it raises json.JSONDecodeError.
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error('Bad JSON: %s' % e)
            return

        # dict.get(key, default) returns the value for key, or default if missing.
        # This is safer than data['key'] which would raise KeyError.
        subject = data.get('subject', 'Cat patrol')
        to_addr = data.get('to', '')
        paths = data.get('paths', [])

        if not to_addr:
            self.get_logger().warning('No "to" address in mail request; skipping')
            return

        # PYTHON CONCEPT: os.environ.get()
        #   Reads an environment variable.  Returns '' (empty string) if
        #   the variable is not set.  This is how we keep secrets OUT of
        #   config files — they live in the shell environment instead.
        host = os.environ.get('CAT_PATROL_SMTP_HOST', '')
        user = os.environ.get('CAT_PATROL_SMTP_USER', '')
        password = os.environ.get('CAT_PATROL_SMTP_PASSWORD', '')
        from_addr = os.environ.get('CAT_PATROL_SMTP_FROM', user)
        port = int(os.environ.get('CAT_PATROL_SMTP_PORT', '587'))

        if not host or not user or not password:
            self.get_logger().warning(
                'SMTP not configured (set CAT_PATROL_SMTP_* env); mail not sent. '
                'Request had %d paths.' % len(paths)
            )
            return

        # ===================================================================
        # Build the email message
        # ===================================================================
        # PYTHON CONCEPT: EmailMessage
        #   Part of Python's built-in `email` package.  It handles:
        #   - MIME encoding (making attachments machine-readable)
        #   - Header formatting (Subject, From, To)
        #   - Multipart structure (text body + file attachments)
        #
        #   set_content() sets the plain-text body.
        #   add_attachment() adds a binary file (JPEG, PDF, etc.).
        m = EmailMessage()
        m['Subject'] = subject
        m['From'] = from_addr
        m['To'] = to_addr
        m.set_content('Cat patrol snapshot batch.\n')

        # Attach each photo file
        for p in paths:
            if not os.path.isfile(p):
                self.get_logger().warning('Missing attachment: %s' % p)
                continue

            # PYTHON CONCEPT: MIME TYPES
            #   mimetypes.guess_type('/path/photo.jpg') → ('image/jpeg', None)
            #   The MIME type tells the email client what kind of file it is.
            #   'image/jpeg' → maintype='image', subtype='jpeg'
            #   If unknown, we fall back to 'application/octet-stream' (generic binary).
            ctype, _ = mimetypes.guess_type(p)
            if ctype is None:
                ctype = 'application/octet-stream'
            maintype, subtype = ctype.split('/', 1)

            # Read the file in binary mode ('rb') and attach it
            with open(p, 'rb') as f:
                m.add_attachment(
                    f.read(),                       # Raw bytes of the file
                    maintype=maintype,               # e.g., 'image'
                    subtype=subtype,                 # e.g., 'jpeg'
                    filename=os.path.basename(p)     # Just the filename, not full path
                )

        # ===================================================================
        # Send via SMTP
        # ===================================================================
        # NETWORKING CONCEPT: SMTP (Simple Mail Transfer Protocol)
        #   SMTP is the standard protocol for sending email.  Steps:
        #     1. Connect to the server (host:port)
        #     2. STARTTLS: upgrade the connection to encrypted (TLS)
        #     3. Login with username and password
        #     4. Send the message
        #
        #   Port 587 = SMTP with STARTTLS (most common for outgoing mail)
        #   Port 465 = SMTP over SSL (older, less common)
        #   Port 25  = Unencrypted SMTP (usually blocked by ISPs)
        #
        #   For Gmail: enable "App Passwords" in your Google Account
        #   (regular password won't work with 2FA enabled).
        #
        # PYTHON CONCEPT: CONTEXT MANAGER ("with ... as s:")
        #   The "with" statement automatically calls s.quit() when the
        #   block exits (even if an exception occurs).  This ensures the
        #   SMTP connection is properly closed.  Same as try/finally but cleaner.
        try:
            with smtplib.SMTP(host, port) as s:
                s.starttls()                # Upgrade to encrypted connection
                s.login(user, password)     # Authenticate
                s.send_message(m)           # Send the email
            self.get_logger().info('Mail sent to %s (%d files).' % (to_addr, len(paths)))
        except Exception as e:
            self.get_logger().error('SMTP failed: %s' % e)


# ===========================================================================
# main — Python ROS 2 node lifecycle
# ===========================================================================
# PYTHON ROS 2 CONCEPT: NODE LIFECYCLE
#   Every Python ROS 2 node follows this pattern:
#     1. rclpy.init()       — initialize the ROS 2 Python client library
#     2. Create node(s)     — instantiate your Node subclass
#     3. rclpy.spin(node)   — block and process callbacks until Ctrl+C
#     4. node.destroy_node() — clean up subscriptions, publishers, timers
#     5. rclpy.shutdown()   — shut down the ROS 2 client library
#
#   rclpy.spin() is the Python equivalent of rclcpp::spin() in C++.
#   It sits in an event loop, waiting for incoming messages and timer events,
#   and dispatches your callbacks.  It only returns when the node is asked
#   to shut down (Ctrl+C, ros2 lifecycle, or rclpy.shutdown from elsewhere).
def main():
    rclpy.init()
    n = MailNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


# PYTHON CONCEPT: if __name__ == '__main__':
#   This guard ensures main() only runs when the script is executed DIRECTLY
#   (python3 mail_node.py), not when it's imported as a module
#   (import mail_node).  It's a Python convention for entry points.
if __name__ == '__main__':
    main()
