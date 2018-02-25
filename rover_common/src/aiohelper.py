import asyncio
from contextlib import suppress


def run_coroutines(*args):
    """
    Runs all of the provided coroutines in an asyncio loop, properly cleaning
    them up when Ctrl+C is pressed.
    """
    with suppress(KeyboardInterrupt):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(asyncio.gather(*args))

        # Once loop has terminated, clean up remaining tasks
        pending = asyncio.Task.all_tasks()
        for task in pending:
            task.cancel()

            # Allow task to terminate gracefully
            with suppress(asyncio.CancelledError):
                loop.run_until_complete(task)


def wait_for(*args):
    """
    Runs the provided coroutines in the primary asyncio loop.
    """
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(*args))


def exec_later(func):
    """
    Adds the provided coroutines to the primary asyncio loop.
    """
    loop = asyncio.get_event_loop()
    loop.create_task(func)
