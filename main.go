package main

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"math/rand"
	"sync"
	"time"

	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"golang.org/x/image/colornames"
)

const (
	WindowWidth  = 1280.0
	WindowHeight = 720.0
	BoidsCount   = 1000
)

// Components
type Position struct {
	x, y float64
}

type Velocity struct {
	dx, dy float64
}

type Boid struct{}

type Entity struct {
	ID       int
	Position *Position
	Velocity *Velocity
	Boid     *Boid
}

type System interface {
	Update(entities []*Entity)
}

// Movement System
type MovementSystem struct{}

func (ms *MovementSystem) Update(entities []*Entity) {
	for _, entity := range entities {
		if entity.Position != nil && entity.Velocity != nil {
			entity.Position.x += entity.Velocity.dx
			entity.Position.y += entity.Velocity.dy
		}
	}
}

// Boids System
type BoidsSystem struct {
	cohesionFactor    float64
	alignmentFactor   float64
	separationFactor  float64
	neighborhoodRange float64
	maxSpeed          float64
}

func (bs *BoidsSystem) Update(entities []*Entity) {
	for _, boid := range entities {
		if boid.Boid == nil {
			continue
		}

		avgPosition := &Position{}
		avgVelocity := &Velocity{}
		avoidance := &Velocity{}
		neighborCount := 0

		for _, other := range entities {
			if other.Boid == nil || boid.ID == other.ID {
				continue
			}
			d := distance(boid.Position, other.Position)
			if d < bs.neighborhoodRange {
				avgPosition.x += other.Position.x
				avgPosition.y += other.Position.y
				avgVelocity.dx += other.Velocity.dx
				avgVelocity.dy += other.Velocity.dy

				separationDistance := bs.neighborhoodRange * 0.3 // Adjust this value as needed
				if d < separationDistance {
					avoidance.dx += boid.Position.x - other.Position.x
					avoidance.dy += boid.Position.y - other.Position.y
				}

				neighborCount++
			}
		}

		if neighborCount > 0 {
			avgPosition.x /= float64(neighborCount)
			avgPosition.y /= float64(neighborCount)
			avgVelocity.dx /= float64(neighborCount)
			avgVelocity.dy /= float64(neighborCount)

			cohesion := &Velocity{
				dx: (avgPosition.x - boid.Position.x) * bs.cohesionFactor,
				dy: (avgPosition.y - boid.Position.y) * bs.cohesionFactor,
			}
			alignment := &Velocity{
				dx: (avgVelocity.dx - boid.Velocity.dx) * bs.alignmentFactor,
				dy: (avgVelocity.dy - boid.Velocity.dy) * bs.alignmentFactor,
			}
			separation := &Velocity{
				dx: avoidance.dx * bs.separationFactor,
				dy: avoidance.dy * bs.separationFactor,
			}

			boid.Velocity.dx += cohesion.dx + alignment.dx + separation.dx
			boid.Velocity.dy += cohesion.dy + alignment.dy + separation.dy

			limitMagnitude(boid.Velocity, bs.maxSpeed)
		}

		const boundaryMargin = 30.0
		const boundaryForce = 0.5

		// If boid is near left boundary
		if boid.Position.x < boundaryMargin {
			boid.Velocity.dx += boundaryForce
		}

		// If boid is near right boundary
		if boid.Position.x > WindowWidth-boundaryMargin {
			boid.Velocity.dx -= boundaryForce
		}

		// If boid is near bottom boundary
		if boid.Position.y < boundaryMargin {
			boid.Velocity.dy += boundaryForce
		}

		// If boid is near top boundary
		if boid.Position.y > WindowHeight-boundaryMargin {
			boid.Velocity.dy -= boundaryForce
		}
	}
}

func steerTowardsCenter(position *Position, windowCenter pixel.Vec) *Velocity {
	const centerForce = 0.001
	return &Velocity{
		dx: (windowCenter.X - position.x) * centerForce,
		dy: (windowCenter.Y - position.y) * centerForce,
	}
}

type World struct {
	Entities []*Entity
	Systems  []System
}

func (w *World) Update() {
	var wg sync.WaitGroup

	for _, system := range w.Systems {
		wg.Add(1) // Increment the WaitGroup counter for each system.

		go func(s System) {
			defer wg.Done() // Decrement the WaitGroup counter once the system finishes updating.

			s.Update(w.Entities)
		}(system)
	}

	wg.Wait() // Wait for all systems to finish updating.
}

func NewWorld() *World {
	return &World{}
}

func distance(p1, p2 *Position) float64 {
	return math.Sqrt(math.Pow(p2.x-p1.x, 2) + math.Pow(p2.y-p1.y, 2))
}

func limitMagnitude(v *Velocity, max float64) {
	mag := math.Sqrt(v.dx*v.dx + v.dy*v.dy)
	if mag > max {
		v.dx = (v.dx / mag) * max
		v.dy = (v.dy / mag) * max
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

	world := NewWorld()

	for i := 0; i < BoidsCount; i++ {
		angle := rand.Float64() * 2 * math.Pi
		speed := rand.Float64()*6 + 2 // speeds between 2 and 8
		boid := &Entity{
			ID:       i,
			Position: &Position{x: rand.Float64() * WindowWidth, y: rand.Float64() * WindowHeight},
			Velocity: &Velocity{dx: math.Cos(angle) * speed, dy: math.Sin(angle) * speed},
			Boid:     &Boid{},
		}
		world.Entities = append(world.Entities, boid)
	}

	boidsSystem := &BoidsSystem{
		cohesionFactor:    0.002,
		alignmentFactor:   0.05,
		separationFactor:  0.1,
		neighborhoodRange: 40,
		maxSpeed:          4,
	}

	world.Systems = append(world.Systems, &MovementSystem{}, boidsSystem)

	for !win.Closed() {
		// Runtime parameter adjustments
		adjustFactor := 0.001 // Adjust by a small amount for fine control
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
				pos := pixel.V(entity.Position.x, entity.Position.y)

				angle := math.Atan2(entity.Velocity.dy, entity.Velocity.dx) + math.Pi/2
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

func createCircleSprite(radius float64) *pixel.Sprite {
	img := image.NewRGBA(image.Rect(0, 0, int(radius*2)+1, int(radius*2)+1))
	cen := pixel.V(radius, radius)
	col := color.RGBA{255, 0, 0, 255} // Red color for our boid circles

	for y := 0.0; y < radius*2; y++ {
		for x := 0.0; x < radius*2; x++ {
			pos := pixel.V(x, y)
			if cen.To(pos).Len() < radius {
				img.Set(int(x), int(y), col)
			}
		}
	}

	pic := pixel.PictureDataFromImage(img)
	return pixel.NewSprite(pic, pic.Bounds())
}

func main() {
	rand.Seed(time.Now().UnixNano())
	pixelgl.Run(run)
}
