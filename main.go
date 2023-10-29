package main

import (
	"boids/vector"
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
	WindowWidth  = 1920.0
	WindowHeight = 1020.0
	BoidsCount   = 5000
)

// Entity is not needed. Boids will be identified by their number from 0 to BoidsCount

// Components
type Vector2D = vector.Vector2D

type Position = Vector2D
type Velocity = Vector2D
type Acceleration = Vector2D

type Rectangle struct {
	Center     Position
	Width      float64
	Height     float64
	HalfWidth  float64
	HalfHeight float64
}

func NewRectangle(center Position, width, height float64) *Rectangle {
	return &Rectangle{
		Center:     center,
		Width:      width,
		Height:     height,
		HalfWidth:  width / 2,
		HalfHeight: height / 2,
	}
}

func (r *Rectangle) Contains(point *Position) bool {
	dx := point.X - r.Center.X
	dy := point.Y - r.Center.Y
	return (dx <= r.HalfWidth && dx >= -r.HalfWidth) &&
		(dy <= r.HalfHeight && dy >= -r.HalfHeight)
}

func (r *Rectangle) Intersects(rangeRec *Rectangle) bool {
	dx := rangeRec.Center.X - r.Center.X
	dy := rangeRec.Center.Y - r.Center.Y
	return (dx <= (r.HalfWidth+rangeRec.HalfWidth) && dx >= -(r.HalfWidth+rangeRec.HalfWidth)) &&
		(dy <= (r.HalfHeight+rangeRec.HalfHeight) && dy >= -(r.HalfHeight+rangeRec.HalfHeight))
}

const capacity = 8 // adjust as needed

var quadtreePool = sync.Pool{
	New: func() interface{} {
		return &Quadtree{
			Components: make([]*int, 0, capacity),
		}
	},
}

type Quadtree struct {
	Boundary       Rectangle
	Components     []*int
	Divided        bool
	NW, NE, SW, SE *Quadtree
}

func NewQuadtree(boundary Rectangle) *Quadtree {
	qt := quadtreePool.Get().(*Quadtree)
	qt.Boundary = boundary
	return qt
}

func (qt *Quadtree) Clear() {
	for i := range qt.Components {
		qt.Components[i] = nil
	}
	qt.Components = qt.Components[:0]
	qt.Divided = false
	if qt.NW != nil {
		qt.NW.Clear()
		quadtreePool.Put(qt.NW)
		qt.NW = nil
	}

	if qt.NE != nil {
		qt.NE.Clear()
		quadtreePool.Put(qt.NE)
		qt.NE = nil
	}

	if qt.SW != nil {
		qt.SW.Clear()
		quadtreePool.Put(qt.SW)
		qt.SW = nil
	}

	if qt.SE != nil {
		qt.SE.Clear()
		quadtreePool.Put(qt.SE)
		qt.SE = nil
	}
}

func (qt *Quadtree) Subdivide() {
	x, y := qt.Boundary.Center.X, qt.Boundary.Center.Y
	w, h := qt.Boundary.Width/2, qt.Boundary.Height/2
	qt.NW = NewQuadtree(*NewRectangle(Vector2D{X: x - w/2, Y: y - h/2}, w, h))
	qt.NE = NewQuadtree(*NewRectangle(Vector2D{X: x + w/2, Y: y - h/2}, w, h))
	qt.SW = NewQuadtree(*NewRectangle(Vector2D{X: x - w/2, Y: y + h/2}, w, h))
	qt.SE = NewQuadtree(*NewRectangle(Vector2D{X: x + w/2, Y: y + h/2}, w, h))
	qt.Divided = true
}

func (qt *Quadtree) Insert(component *Position, i *int) bool {
	if !qt.Boundary.Contains(component) {
		return false
	}

	if len(qt.Components) < capacity {
		qt.Components = append(qt.Components, i)
		return true
	}

	if !qt.Divided {
		qt.Subdivide()
	}

	return qt.NW.Insert(component, i) || qt.NE.Insert(component, i) || qt.SW.Insert(component, i) || qt.SE.Insert(component, i)
}

func (qt *Quadtree) Query(rangeRec *Rectangle, foundComponents []*int) []*int {
	if !qt.Boundary.Intersects(rangeRec) {
		return foundComponents
	}

	for _, component := range qt.Components {
		if rangeRec.Contains(&positionComponents[*component]) {
			foundComponents = append(foundComponents, component)
		}
	}

	if qt.Divided {
		foundComponents = qt.NW.Query(rangeRec, foundComponents)
		foundComponents = qt.NE.Query(rangeRec, foundComponents)
		foundComponents = qt.SW.Query(rangeRec, foundComponents)
		foundComponents = qt.SE.Query(rangeRec, foundComponents)
	}

	return foundComponents
}

// Systems
type MovementSystem struct {
	maxSpeed float64
}

func (ms *MovementSystem) Update() {
	for i := 0; i < BoidsCount; i++ {
		positionComponents[i].Add(&velocityComponents[i])
		velocityComponents[i].Add(&accelerationComponents[i])
		velocityComponents[i].Limit(ms.maxSpeed)
		accelerationComponents[i].Multiply(0)
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

var alignmentSteering = Vector2D{}
var cohesionSteering = Vector2D{}
var separationSteering = Vector2D{}

func (bs *SteeringSystem) Update() {
	for i := 0; i < BoidsCount; i++ {

		alignmentSteering.Multiply(0)
		cohesionSteering.Multiply(0)
		separationSteering.Multiply(0)

		var neighborCount float64 = 0.0

		rangeRec := NewRectangle(
			positionComponents[i],
			bs.neighborhoodRange,
			bs.neighborhoodRange,
		)

		neighbours := qt.Query(rangeRec, nil)

		for _, j := range neighbours {
			var otherPosition = &positionComponents[*j]
			var otherVelocity = &velocityComponents[*j]

			if i == *j {
				continue
			}

			d := positionComponents[i].Distance(otherPosition)
			alignmentSteering.Add(otherVelocity)
			cohesionSteering.Add(otherPosition)

			diff := positionComponents[i]
			diff.Subtract(otherPosition)
			diff.Divide(d) // Not squared?
			separationSteering.Add(&diff)

			neighborCount++
		}

		if neighborCount > 0 {
			alignmentSteering.Divide(neighborCount)
			alignmentSteering.SetMagnitude(bs.maxSpeed)
			alignmentSteering.Subtract(&velocityComponents[i])
			alignmentSteering.Limit(bs.maxForce)

			cohesionSteering.Divide(neighborCount)
			cohesionSteering.Subtract(&positionComponents[i])
			cohesionSteering.SetMagnitude(bs.maxSpeed)
			cohesionSteering.Subtract(&velocityComponents[i])
			cohesionSteering.Limit(bs.maxForce)

			separationSteering.Divide(neighborCount)
			separationSteering.SetMagnitude(bs.maxSpeed)
			separationSteering.Subtract(&velocityComponents[i])
			separationSteering.SetMagnitude(bs.maxForce)

			alignmentSteering.Multiply(bs.alignmentFactor)
			cohesionSteering.Multiply(bs.cohesionFactor)
			separationSteering.Multiply(bs.separationFactor)

			accelerationComponents[i].Add(&alignmentSteering)
			accelerationComponents[i].Add(&cohesionSteering)
			accelerationComponents[i].Add(&separationSteering)
			accelerationComponents[i].Divide(3) // WHY?
		}

		// If boid crosses left boundary
		if positionComponents[i].X < 0 {
			positionComponents[i].X = WindowWidth
		}

		// If boid crosses right boundary
		if positionComponents[i].X > WindowWidth {
			positionComponents[i].X = 0
		}

		// If boid crosses bottom boundary
		if positionComponents[i].Y < 0 {
			positionComponents[i].Y = WindowHeight
		}

		// If boid crosses top boundary
		if positionComponents[i].Y > WindowHeight {
			positionComponents[i].Y = 0
		}
	}
}

var ids [BoidsCount]int
var positionComponents [BoidsCount]Position
var velocityComponents [BoidsCount]Position
var accelerationComponents [BoidsCount]Position
var qt *Quadtree

func run() {
	cfg := pixelgl.WindowConfig{
		Title:  "Boids Simulation",
		Bounds: pixel.R(0, 0, WindowWidth, WindowHeight),
	}

	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	for i := 0; i < BoidsCount; i++ {
		angle := rand.Float64() * 2 * math.Pi
		speed := rand.Float64()
		ids[i] = i
		positionComponents[i] = Vector2D{X: rand.Float64() * WindowWidth, Y: rand.Float64() * WindowHeight}
		velocityComponents[i] = Vector2D{X: math.Cos(angle) * speed, Y: math.Sin(angle) * speed}
		accelerationComponents[i] = Vector2D{X: 0, Y: 0}
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

	boidSprite := createIsoscelesTriangleSprite(5, 10) // Adjust baseLength and height as needed

	batch := pixel.NewBatch(&pixel.TrianglesData{}, boidSprite.Picture())

	boundary := NewRectangle(
		vector.Vector2D{X: WindowWidth / 2, Y: WindowHeight / 2},
		WindowWidth,
		WindowHeight,
	)

	for !win.Closed() {
		start := time.Now()

		// Runtime parameter adjustments
		adjustFactor := 0.1
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

		adjustSpeed := 0.1
		if win.JustPressed(pixelgl.KeyR) {
			steeringSystem.maxSpeed += adjustSpeed
			fmt.Printf("Max Speed: %f\n", steeringSystem.maxSpeed)
		} else if win.JustPressed(pixelgl.KeyF) {
			steeringSystem.maxSpeed -= adjustSpeed
			fmt.Printf("Max Speed: %f\n", steeringSystem.maxSpeed)
		}

		win.Clear(colornames.Black)

		UpdateSprites(boidSprite, batch, win)
		batch.Clear()

		qt = NewQuadtree(*boundary)

		for i := 0; i < BoidsCount; i++ {
			qt.Insert(&positionComponents[i], &ids[i])
		}

		steeringSystem.Update()
		movementSystem.Update()

		qt.Clear()

		win.Update()

		elapsed := time.Since(start)
		fmt.Printf("Iteration took %d ms\n", elapsed.Milliseconds())
	}
}

func UpdateSprites(boidSprite *pixel.Sprite, batch *pixel.Batch, win *pixelgl.Window) {
	for i := 0; i < BoidsCount; i++ {
		pos := pixel.V(positionComponents[i].X, positionComponents[i].Y)

		angle := math.Atan2(velocityComponents[i].Y, velocityComponents[i].X) + math.Pi/2
		mat := pixel.IM.Moved(pos).Rotated(pos, angle)

		boidSprite.Draw(batch, mat)
	}
	batch.Draw(win)
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

	// p, err := os.Create("myprogram.prof")
	// if err != nil {

	// 	fmt.Println(err)
	// 	return
	// }
	// pprof.StartCPUProfile(p)
	// defer pprof.StopCPUProfile()

	// t, err := os.Create("myprogram.trace")
	// if err != nil {

	// 	fmt.Println(err)
	// 	return
	// }
	// trace.Start(t)
	// defer trace.Stop()

	pixelgl.Run(run)
}
