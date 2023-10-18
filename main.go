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
	BoidsCount   = 100
)

// Components
type Vector2D = vector.Vector2D

type Boid struct{}

type Entity struct {
	ID           int
	Position     Vector2D
	Velocity     Vector2D
	Acceleration Vector2D
	Boid         *Boid
}

type System interface {
	Update(entities []*Entity)
}

// Movement System
type MovementSystem struct {
	maxSpeed float64
}

func (ms *MovementSystem) Update(entities []*Entity) {
	for _, entity := range entities {
		entity.Position.Add(entity.Velocity)
		entity.Velocity.Add(entity.Acceleration)
		entity.Velocity.Limit(ms.maxSpeed)
		entity.Acceleration.Multiply(0)
	}
}

// Boids System
type BoidsSystem struct {
	cohesionFactor    float64
	alignmentFactor   float64
	separationFactor  float64
	neighborhoodRange float64
	maxForce          float64
	maxSpeed          float64
}

func (bs *BoidsSystem) Update(entities []*Entity) {
	for _, boid := range entities {
		if boid.Boid == nil {
			continue
		}

		alignmentSteering := Vector2D{}
		cohesionSteering := Vector2D{}
		separationSteering := Vector2D{}
		var neighborCount float64 = 0.0

		for _, other := range entities {
			if other.Boid == nil || boid.ID == other.ID {
				continue
			}
			d := boid.Position.Distance(other.Position)
			if d < bs.neighborhoodRange {
				alignmentSteering.Add(other.Velocity)
				cohesionSteering.Add(other.Position)

				diff := boid.Position
				diff.Subtract(other.Position)
				diff.Divide(d) // Not squared?
				separationSteering.Add(diff)

				neighborCount++
			}
		}

		if neighborCount > 0 {

			alignmentSteering.Divide(neighborCount)
			alignmentSteering.SetMagnitude(bs.maxSpeed)
			alignmentSteering.Subtract(boid.Velocity)
			alignmentSteering.Limit(bs.maxForce)

			cohesionSteering.Divide(neighborCount)
			cohesionSteering.Subtract(boid.Position)
			cohesionSteering.SetMagnitude(bs.maxSpeed)
			cohesionSteering.Subtract(boid.Velocity)
			cohesionSteering.Limit(bs.maxForce)

			separationSteering.Divide(neighborCount)
			separationSteering.SetMagnitude(bs.maxSpeed)
			separationSteering.Subtract(boid.Velocity)
			separationSteering.SetMagnitude(bs.maxForce)

			alignmentSteering.Multiply(bs.alignmentFactor)
			cohesionSteering.Multiply(bs.cohesionFactor)
			separationSteering.Multiply(bs.separationFactor)

			boid.Acceleration.Add(alignmentSteering)
			boid.Acceleration.Add(cohesionSteering)
			boid.Acceleration.Add(separationSteering)
			boid.Acceleration.Divide(3) // WHY?
		}

		const boundaryMargin = 50.0
		const boundaryForce = 0.2

		const bounce = false

		if bounce {
			// If boid is near left boundary
			if boid.Position.X < boundaryMargin {
				boid.Velocity.X += boundaryForce
			}

			// If boid is near right boundary
			if boid.Position.X > WindowWidth-boundaryMargin {
				boid.Velocity.X -= boundaryForce
			}

			// If boid is near bottom boundary
			if boid.Position.Y < boundaryMargin {
				boid.Velocity.Y += boundaryForce
			}

			// If boid is near top boundary
			if boid.Position.Y > WindowHeight-boundaryMargin {
				boid.Velocity.Y -= boundaryForce
			}
		} else {
			// If boid crosses left boundary
			if boid.Position.X < 0 {
				boid.Position.X = WindowWidth
			}

			// If boid crosses right boundary
			if boid.Position.X > WindowWidth {
				boid.Position.X = 0
			}

			// If boid crosses bottom boundary
			if boid.Position.Y < 0 {
				boid.Position.Y = WindowHeight
			}

			// If boid crosses top boundary
			if boid.Position.Y > WindowHeight {
				boid.Position.Y = 0
			}
		}
	}
}

type World struct {
	Entities []*Entity
	Systems  []System
}

func (w *World) Update() {
	for _, system := range w.Systems {
		system.Update(w.Entities)
	}
}

func NewWorld() *World {
	return &World{}
}

func distance(p1, p2 *Vector2D) float64 {
	return math.Sqrt(math.Pow(p2.X-p1.X, 2) + math.Pow(p2.Y-p1.Y, 2))
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

	world := NewWorld()

	for i := 0; i < BoidsCount; i++ {
		angle := rand.Float64() * 2 * math.Pi
		speed := rand.Float64()
		boid := &Entity{
			ID:           i,
			Position:     Vector2D{X: rand.Float64() * WindowWidth, Y: rand.Float64() * WindowHeight},
			Velocity:     Vector2D{X: math.Cos(angle) * speed, Y: math.Sin(angle) * speed},
			Acceleration: Vector2D{X: 0, Y: 0},
			Boid:         &Boid{},
		}
		world.Entities = append(world.Entities, boid)
	}

	movementSystem := &MovementSystem{
		maxSpeed: 1,
	}

	boidsSystem := &BoidsSystem{
		cohesionFactor:    0.9,
		alignmentFactor:   1.0,
		separationFactor:  1.2,
		neighborhoodRange: 75,
		maxForce:          1.0,
		maxSpeed:          4,
	}

	world.Systems = append(world.Systems, movementSystem, boidsSystem)

	for !win.Closed() {
		// Runtime parameter adjustments
		adjustFactor := 0.01 // Adjust by a small amount for fine control
		if win.JustPressed(pixelgl.KeyW) {
			boidsSystem.cohesionFactor += adjustFactor
			fmt.Printf("Cohesion Factor: %f\n", boidsSystem.cohesionFactor)
		} else if win.JustPressed(pixelgl.KeyS) {
			boidsSystem.cohesionFactor -= adjustFactor
			fmt.Printf("Cohesion Factor: %f\n", boidsSystem.cohesionFactor)
		}

		if win.JustPressed(pixelgl.KeyA) {
			boidsSystem.alignmentFactor += adjustFactor
			fmt.Printf("Alignment Factor: %f\n", boidsSystem.alignmentFactor)
		} else if win.JustPressed(pixelgl.KeyD) {
			boidsSystem.alignmentFactor -= adjustFactor
			fmt.Printf("Alignment Factor: %f\n", boidsSystem.alignmentFactor)
		}

		if win.JustPressed(pixelgl.KeyQ) {
			boidsSystem.separationFactor += adjustFactor
			fmt.Printf("Separation Factor: %f\n", boidsSystem.separationFactor)
		} else if win.JustPressed(pixelgl.KeyE) {
			boidsSystem.separationFactor -= adjustFactor
			fmt.Printf("Separation Factor: %f\n", boidsSystem.separationFactor)
		}

		adjustRange := 1.0 // Adjust range by a bit larger amount
		if win.JustPressed(pixelgl.KeyZ) {
			boidsSystem.neighborhoodRange += adjustRange
			fmt.Printf("Neighborhood Range: %f\n", boidsSystem.neighborhoodRange)
		} else if win.JustPressed(pixelgl.KeyX) {
			boidsSystem.neighborhoodRange -= adjustRange
			fmt.Printf("Neighborhood Range: %f\n", boidsSystem.neighborhoodRange)
		}

		adjustSpeed := 0.1 // Adjust speed in slightly larger increments
		if win.JustPressed(pixelgl.KeyR) {
			boidsSystem.maxSpeed += adjustSpeed
			fmt.Printf("Max Speed: %f\n", boidsSystem.maxSpeed)
		} else if win.JustPressed(pixelgl.KeyF) {
			boidsSystem.maxSpeed -= adjustSpeed
			fmt.Printf("Max Speed: %f\n", boidsSystem.maxSpeed)
		}

		win.Clear(colornames.Black)

		boidSprite := createIsoscelesTriangleSprite(10, 20) // Adjust baseLength and height as needed
		for _, entity := range world.Entities {
			if entity.Boid != nil {
				pos := pixel.V(entity.Position.X, entity.Position.Y)

				angle := math.Atan2(entity.Velocity.Y, entity.Velocity.X) + math.Pi/2
				mat := pixel.IM.Moved(pos).Rotated(pos, angle)

				boidSprite.Draw(win, mat)
			}
		}

		world.Update()
		win.Update()
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
