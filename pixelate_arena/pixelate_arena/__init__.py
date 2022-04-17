from gym.envs.registration import register

register(
    id='pixelate_arena-v0',
    entry_point='pixelate_arena.envs:PixelateArena',
)