#!/usr/bin/env python3
"""mode_db_writer.py: подписывается на /control/mode и пишет в БД current_mode"""
import os
import asyncio
import threading

import asyncpg
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt8

class ModeWriter(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('mode_writer')
        self.pool: asyncpg.Pool | None = None
        self._loop = loop
        # подписка на ROS-топик
        self.create_subscription(UInt8, '/control/mode', self.on_mode, 10)

    async def ainit(self):
        self.pool = await asyncpg.create_pool(dsn=os.environ['DB_DSN'])
        self.get_logger().info('Connected to DB')

    def on_mode(self, msg: UInt8):
        mode = int(msg.data)
        self.get_logger().info(f'Received mode: {mode}, scheduling DB write')
        # без get_event_loop() — используем заранее сохранённый loop
        self._loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self.write_db(mode))
        )

    async def write_db(self, mode: int):
        assert self.pool is not None, 'DB pool not initialized'
        async with self.pool.acquire() as conn:
            await conn.execute('UPDATE current_mode SET mode=$1', mode)
        self.get_logger().info(f'Mode {mode} written to DB')


def main():
    # создаём и регистрируем asyncio-loop до rclpy
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    rclpy.init()
    node = ModeWriter(loop)

    # инициализация DB pool
    loop.run_until_complete(node.ainit())

    # spin ROS колбэков в отдельном потоке
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # основной asyncio-loop отвечает за запись в БД
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        loop.stop()

if __name__ == '__main__':
    main()
