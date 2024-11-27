include("../src/main.jl")
using Test

function test_get_vel()
    speed = 1.25

    # Testing forward
    v = get_vel(0.0, 0.0, speed)
    @test v[1] ≈ speed atol = 1e-15

    # Testing 90
    v = get_vel(pi / 2, 0.0, speed)
    @test v[1] ≈ 0.0 atol = 1e-15
    @test v[2] ≈ speed atol = 1e-15

    # Testing backward
    v = get_vel(pi, 0.0, speed)
    @test v[1] ≈ -speed atol = 1e-15

    # Testing 270
    v = get_vel(3pi / 2, 0.0, speed)
    @test v[1] ≈ 0.0 atol = 1e-15
    @test v[2] ≈ -speed atol = 1e-15

    # Testing 30
    v = get_vel(pi / 6, 0.0, speed)
    @test v[1] ≈ cos(pi / 6) * speed atol = 1e-15
    @test v[2] ≈ sin(pi / 6) * speed atol = 1e-15

    # Testing magnitude
    v = get_vel(pi / 6, pi / 3, speed)
    @test norm(v) ≈ speed atol = 1e-15
end

function test_angle_between_2()
    # Testing cardinals
    v1 = [1, 0]
    v2 = [0, 1]

    @test angle_between_2(v1, v1) ≈ 0.0 atol = 1e-15
    @test angle_between_2(v2, v2) ≈ 0.0 atol = 1e-15
    @test angle_between_2(v1, v2) ≈ pi / 2 atol = 1e-15
    @test angle_between_2(v2, v1) ≈ -pi / 2 atol = 1e-15
    @test abs(angle_between_2(v1, -v1)) ≈ pi atol = 1e-15
    @test abs(angle_between_2(v2, -v2)) ≈ pi atol = 1e-15

    # Positive angles 
    θ1 = pi / 6
    θ2 = -pi / 2.9
    m1 = 3.2
    m2 = 8.2

    v1 = [m1 * cos(θ1), m1 * sin(θ1)]
    v2 = [m2 * cos(θ2), m2 * sin(θ2)]
    @test angle_between_2(v1, v2) ≈ θ2 - θ1 atol = 1e-15
    @test angle_between_2(v2, v1) ≈ θ1 - θ2 atol = 1e-15
end

function test_relative_travel_direction()
    u1 = [1.0, 0.0]
    u2 = [-1.0, 0.0]
    u3 = [0.0, 1.0]
    u4 = [0.0, -1.0]
    u5 = [1.0, 1.5]
    u6 = [1.0, -1.5]
    u7 = [-1.5, 1.0]
    u8 = [-1.0, -1.0]

    # Horizontal
    v1 = [1.0, 0.0]
    @test get_relative_travel_direction(v1, u1) == prograde
    @test get_relative_travel_direction(v1, u2) == retrograde
    @test get_relative_travel_direction(v1, u3) == left
    @test get_relative_travel_direction(v1, u4) == right
    @test get_relative_travel_direction(v1, u5) == left
    @test get_relative_travel_direction(v1, u6) == right
    @test get_relative_travel_direction(v1, u7) == retrograde
    @test get_relative_travel_direction(v1, u8) == retrograde

    # Vertical 
    v2 = [0.0, 1.0]
    @test get_relative_travel_direction(v2, u1) == right
    @test get_relative_travel_direction(v2, u2) == left
    @test get_relative_travel_direction(v2, u3) == prograde
    @test get_relative_travel_direction(v2, u4) == retrograde
    @test get_relative_travel_direction(v2, u5) == prograde
    @test get_relative_travel_direction(v2, u6) == retrograde
    @test get_relative_travel_direction(v2, u7) == left
    @test get_relative_travel_direction(v2, u8) == retrograde

end

function test_is_in_volume()
    l = [1, 1, 1]
    h = [2, 2, 2]

    pin = [
        [1, 1, 1],
        [2, 1, 1],
        [1, 2, 1],
        [1, 1, 2],
        [2, 2, 1],
        [2, 1, 2],
        [1, 2, 2],
        [2, 2, 2],
    ]

    pout = [
        [0.9, 1, 1],
        [1, 0.9, 1],
        [1, 1, 0.9],
        [2.1, 1, 1],
        [1, 2.1, 1],
        [1, 1, 2.1],
    ]

    for valid in pin
        @test is_in_volume(valid, l, h) == true
    end

    for invalid in pout
        @test is_in_volume(invalid, l, h) != true
    end

    @test true
end

