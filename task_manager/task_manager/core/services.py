from std_srvs.srv import Trigger, SetBool
from .states import TaskState

class ServiceManager:
    def __init__(self, node, state_machine):
        self.node = node
        self.state_machine = state_machine
        self.setup_services()

    def setup_services(self):
        self.node.create_service(
            Trigger,
            'move_to_home',
            self.move_to_home_callback,
            callback_group=self.node.service_group
        )
        self.node.create_service(
            Trigger,
            'task_control',
            self.task_control_callback,
            callback_group=self.node.service_group
        )
        self.node.create_service(
            SetBool,
            'pause_task',
            self.pause_task_callback,
            callback_group=self.node.service_group
        )

    def move_to_home_callback(self, request, response):
        if self.state_machine.state != TaskState.IDLE:
            response.success = False
            response.message = f'Cannot move home: task is in state {self.state_machine.state.name}'
            return response

        self.node.publish_status("Moving to home position")
        home_pos_str = ','.join(map(str, self.state_machine.home_position))
        self.node.send_robot_command(f"move_joints:{home_pos_str}")

        response.success = True
        response.message = 'Moving to home position'
        return response

    def task_control_callback(self, request, response):
        if self.state_machine.state != TaskState.IDLE:
            response.success = False
            response.message = f'Task already active in state {self.state_machine.state.name}'
            return response

        success = self.node.start_task()
        response.success = success
        response.message = 'Task started' if success else 'Failed to start task'
        return response

    def pause_task_callback(self, request, response):
        if request.data:  # Pause
            if self.state_machine.state == TaskState.IDLE or self.state_machine.state == TaskState.PAUSED:
                response.success = False
                response.message = f'Cannot pause: task is in state {self.state_machine.state.name}'
                return response

            self.state_machine._paused_state = self.state_machine.state  # Save current state for resuming later
            self.state_machine.transition_to(TaskState.PAUSED)
            self.node.send_robot_command("stop")  # Stop current movement
            response.success = True
            response.message = 'Task paused'
        else:  # Resume
            if self.state_machine.state != TaskState.PAUSED:
                response.success = False
                response.message = f'Cannot resume: task is not paused (state: {self.state_machine.state.name})'
                return response

            if hasattr(self.state_machine, '_paused_state'):  # Restore saved state
                self.state_machine.transition_to(self.state_machine._paused_state)
                delattr(self.state_machine, '_paused_state')
                response.success = True
                response.message = 'Task resumed'
            else:
                self.state_machine.transition_to(TaskState.RETURNING_HOME)  # Fallback
                response.success = True
                response.message = 'Task resumed (returning home)'
        return response