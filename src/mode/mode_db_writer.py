# mode_db_writer.py  (отдельный docker-сервис, 30 строк кода)
import asyncpg, rclpy, asyncio
from rclpy.node import Node
from std_msgs.msg import UInt8
import os

class ModeWriter(Node):
    def __init__(self):
        super().__init__("mode_writer")
        self.pool = None
        self.create_subscription(UInt8, "/control/mode", self.on_mode, 10)

    async def ainit(self):
        self.pool = await asyncpg.create_pool(dsn=os.environ["DB_DSN"])

    def on_mode(self, msg):
        asyncio.get_event_loop().create_task(self.write(int(msg.data)))

    async def write(self, mode):
        async with self.pool.acquire() as conn:
            await conn.execute("UPDATE current_mode SET mode=$1", mode)

def main():
    rclpy.init()
    node = ModeWriter()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.ainit())
    rclpy.spin(node)
    from rclpy.executors import AsyncIOExecutor
    executor = AsyncIOExecutor()
    executor.add_node(node)
    try:
        loop.run_until_complete(executor.spin())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()