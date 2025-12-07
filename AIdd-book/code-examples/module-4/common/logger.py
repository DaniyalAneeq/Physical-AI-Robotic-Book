from rclpy.logging import get_logger, Logger

# Get a module-level logger for use in non-Node classes or scripts.
# It's assumed that rclpy.init() will be called by a main ROS 2 node or entry point
# before any logging functions from this module are used.
_module_logger: Logger = get_logger('vla_module_logger')

def get_shared_logger() -> Logger:
    """
    Returns a shared rclpy logger instance for use outside of rclpy.Node instances.

    This is useful for utility scripts or classes that are part of a ROS 2 project
    but do not inherit from rclpy.node.Node.

    Returns:
        The shared Logger instance.
    """
    return _module_logger

# Convenience functions for logging
def info(message: str):
    """Logs an informational message using the shared logger."""
    _module_logger.info(message)

def warn(message: str):
    """Logs a warning message using the shared logger."""
    _module_logger.warn(message)

def error(message: str):
    """Logs an error message using the shared logger."""
    _module_logger.error(message)

if __name__ == '__main__':
    # This block is for demonstration purposes.
    # In a real ROS 2 setup, a main node would call rclpy.init(), and then
    # other parts of the application could use this shared logger.
    print("To use this logger, ensure rclpy.init() is called by your main ROS 2 application.")
    print("Example from another script:")
    print("from AIdd_book.code_examples.module_4.common import logger")
    print("logger.info('Test info message from shared logger.')")
