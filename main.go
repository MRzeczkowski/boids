package main

import (
	"boids/vector"
	"fmt"
	"image"
	"image/color"
	"math"
	"math/rand"
	"time"

	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"golang.org/x/image/colornames"
)

const (
	WindowWidth  = 1440.0
	WindowHeight = 900.0
	BoidsCount   = 1000
)

type Entity struct {
	ID int
}

// Components
type Vector2D = vector.Vector2D

type MovementComponent struct { // Composite component for better data locality - see MovementSystem.Update for example
	Position     Vector2D
	Velocity     Vector2D
	Acceleration Vector2D
}

// Systems
type MovementSystem struct {
	maxSpeed float64
}

func (ms *MovementSystem) Update(movementComponents *[BoidsCount]MovementComponent) {
	for i := range movementComponents {
		var m = &movementComponents[i]
		m.Position.Add(m.Velocity)
		m.Velocity.Add(m.Acceleration)
		m.Velocity.Limit(ms.maxSpeed)
		m.Acceleration.Multiply(0)
	}
}

type SteeringSystem struct {
	cohesionFactor    float64
	alignmentFactor   float64
	separationFactor  float64
	neighborhoodRange float64
	maxForce          float64
	maxSpeed          float64
}

func (bs *SteeringSystem) Update(movementComponents *[BoidsCount]MovementComponent) {
	for i := range movementComponents {

		var current = &movementComponents[i]
		alignmentSteering := Vector2D{}
		cohesionSteering := Vector2D{}
		separationSteering := Vector2D{}
		var neighborCount float64 = 0.0

		for j := range movementComponents {
			var other = &movementComponents[j]
			if current == other {
				continue
			}
			d := current.Position.Distance(other.Position)
			if d < bs.neighborhoodRange {
				alignmentSteering.Add(other.Velocity)
				cohesionSteering.Add(other.Position)

				diff := current.Position
				diff.Subtract(other.Position)
				diff.Divide(d) // Not squared?
				separationSteering.Add(diff)

				neighborCount++
			}
		}

		if neighborCount > 0 {

			alignmentSteering.Divide(neighborCount)
			alignmentSteering.SetMagnitude(bs.maxSpeed)
			alignmentSteering.Subtract(current.Velocity)
			alignmentSteering.Limit(bs.maxForce)

			cohesionSteering.Divide(neighborCount)
			cohesionSteering.Subtract(current.Position)
			cohesionSteering.SetMagnitude(bs.maxSpeed)
			cohesionSteering.Subtract(current.Velocity)
			cohesionSteering.Limit(bs.maxForce)

			separationSteering.Divide(neighborCount)
			separationSteering.SetMagnitude(bs.maxSpeed)
			separationSteering.Subtract(current.Velocity)
			separationSteering.SetMagnitude(bs.maxForce)

			alignmentSteering.Multiply(bs.alignmentFactor)
			cohesionSteering.Multiply(bs.cohesionFactor)
			separationSteering.Multiply(bs.separationFactor)

			current.Acceleration.Add(alignmentSteering)
			current.Acceleration.Add(cohesionSteering)
			current.Acceleration.Add(separationSteering)
			current.Acceleration.Divide(3) // WHY?
		}

		const boundaryMargin = 50.0
		const boundaryForce = 0.2

		const bounce = false

		if bounce {
			// If boid is near left boundary
			if current.Position.X < boundaryMargin {
				current.Velocity.X += boundaryForce
			}

			// If boid is near right boundary
			if current.Position.X > WindowWidth-boundaryMargin {
				current.Velocity.X -= boundaryForce
			}

			// If boid is near bottom boundary
			if current.Position.Y < boundaryMargin {
				current.Velocity.Y += boundaryForce
			}

			// If boid is near top boundary
			if current.Position.Y > WindowHeight-boundaryMargin {
				current.Velocity.Y -= boundaryForce
			}
		} else {
			// If boid crosses left boundary
			if current.Position.X < 0 {
				current.Position.X = WindowWidth
			}

			// If boid crosses right boundary
			if current.Position.X > WindowWidth {
				current.Position.X = 0
			}

			// If boid crosses bottom boundary
			if current.Position.Y < 0 {
				current.Position.Y = WindowHeight
			}

			// If boid crosses top boundary
			if current.Position.Y > WindowHeight {
				current.Position.Y = 0
			}
		}
	}
}

func run() {
	cfg := pixelgl.WindowConfig{
		Title:  "Boids Simulation",
		Bounds: pixel.R(0, 0, WindowWidth, WindowHeight),
	}

	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	var entities [BoidsCount]Entity
	var movementComponents [BoidsCount]MovementComponent

	for i := 0; i < BoidsCount; i++ {
		angle := rand.Float64() * 2 * math.Pi
		speed := rand.Float64()
		entities[i] = Entity{ID: i}
		movementComponents[i] = MovementComponent{
			Position:     Vector2D{X: rand.Float64() * WindowWidth, Y: rand.Float64() * WindowHeight},
			Velocity:     Vector2D{X: math.Cos(angle) * speed, Y: math.Sin(angle) * speed},
			Acceleration: Vector2D{X: 0, Y: 0},
		}
	}

	var movementSystem = MovementSystem{
		maxSpeed: 1,
	}

	var steeringSystem = SteeringSystem{
		cohesionFactor:    0.9,
		alignmentFactor:   1.0,
		separationFactor:  1.2,
		neighborhoodRange: 75,
		maxForce:          1.0,
		maxSpeed:          4,
	}

	for !win.Closed() {
		start := time.Now()

		// Runtime parameter adjustments
		adjustFactor := 0.01 // Adjust by a small amount for fine control
		if win.JustPressed(pixelgl.KeyW) {
			steeringSystem.cohesionFactor += adjustFactor
			fmt.Printf("Cohesion Factor: %f\n", steeringSystem.cohesionFactor)
		} else if win.JustPressed(pixelgl.KeyS) {
			steeringSystem.cohesionFactor -= adjustFactor
			fmt.Printf("Cohesion Factor: %f\n", steeringSystem.cohesionFactor)
		}

		if win.JustPressed(pixelgl.KeyA) {
			steeringSystem.alignmentFactor += adjustFactor
			fmt.Printf("Alignment Factor: %f\n", steeringSystem.alignmentFactor)
		} else if win.JustPressed(pixelgl.KeyD) {
			steeringSystem.alignmentFactor -= adjustFactor
			fmt.Printf("Alignment Factor: %f\n", steeringSystem.alignmentFactor)
		}

		if win.JustPressed(pixelgl.KeyQ) {
			steeringSystem.separationFactor += adjustFactor
			fmt.Printf("Separation Factor: %f\n", steeringSystem.separationFactor)
		} else if win.JustPressed(pixelgl.KeyE) {
			steeringSystem.separationFactor -= adjustFactor
			fmt.Printf("Separation Factor: %f\n", steeringSystem.separationFactor)
		}

		adjustRange := 1.0 // Adjust range by a bit larger amount
		if win.JustPressed(pixelgl.KeyZ) {
			steeringSystem.neighborhoodRange += adjustRange
			fmt.Printf("Neighborhood Range: %f\n", steeringSystem.neighborhoodRange)
		} else if win.JustPressed(pixelgl.KeyX) {
			steeringSystem.neighborhoodRange -= adjustRange
			fmt.Printf("Neighborhood Range: %f\n", steeringSystem.neighborhoodRange)
		}

		adjustSpeed := 0.1 // Adjust speed in slightly larger increments
		if win.JustPressed(pixelgl.KeyR) {
			steeringSystem.maxSpeed += adjustSpeed
			fmt.Printf("Max Speed: %f\n", steeringSystem.maxSpeed)
		} else if win.JustPressed(pixelgl.KeyF) {
			steeringSystem.maxSpeed -= adjustSpeed
			fmt.Printf("Max Speed: %f\n", steeringSystem.maxSpeed)
		}

		win.Clear(colornames.Black)

		boidSprite := createIsoscelesTriangleSprite(10, 20) // Adjust baseLength and height as needed
		for i := range movementComponents {
			pos := pixel.V(movementComponents[i].Position.X, movementComponents[i].Position.Y)

			angle := math.Atan2(movementComponents[i].Velocity.Y, movementComponents[i].Velocity.X) + math.Pi/2
			mat := pixel.IM.Moved(pos).Rotated(pos, angle)

			boidSprite.Draw(win, mat)
		}

		steeringSystem.Update(&movementComponents)
		movementSystem.Update(&movementComponents)
		win.Update()

		elapsed := time.Since(start)
		fmt.Printf("Iteration took %d ms\n", elapsed.Milliseconds())
	}
}

func createIsoscelesTriangleSprite(baseLength, height float64) *pixel.Sprite {
	img := image.NewRGBA(image.Rect(0, 0, int(baseLength), int(height)))
	col := color.RGBA{255, 0, 0, 255} // Red color for our boid triangles

	// Points of the triangle
	top := pixel.V(baseLength/2, height)
	left := pixel.V(0, 0)
	right := pixel.V(baseLength, 0)

	// Draw the triangle onto the image
	for x := 0.0; x <= baseLength; x++ {
		for y := 0.0; y <= height; y++ {
			pos := pixel.V(x, y)
			if insideTriangle(pos, top, left, right) {
				img.Set(int(x), int(y), col)
			}
		}
	}

	pic := pixel.PictureDataFromImage(img)
	return pixel.NewSprite(pic, pic.Bounds())
}

func insideTriangle(pt, v1, v2, v3 pixel.Vec) bool {
	d1 := sign(pt, v1, v2)
	d2 := sign(pt, v2, v3)
	d3 := sign(pt, v3, v1)

	hasNeg := (d1 < 0) || (d2 < 0) || (d3 < 0)
	hasPos := (d1 > 0) || (d2 > 0) || (d3 > 0)

	return !(hasNeg && hasPos)
}

func sign(p1, p2, p3 pixel.Vec) float64 {
	return (p1.X-p3.X)*(p2.Y-p3.Y) - (p2.X-p3.X)*(p1.Y-p3.Y)
}

func main() {
	rand.Seed(time.Now().UnixNano())
	pixelgl.Run(run)
}
