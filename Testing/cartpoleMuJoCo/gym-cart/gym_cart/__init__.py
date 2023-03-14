from gym.envs.registration import register

register(
    id='gym_cart/MyCartPole-v0',
    entry_point='gym_cart.envs:MyCartPole',
    max_episode_steps=1000,
    reward_threshold=950.0,
)