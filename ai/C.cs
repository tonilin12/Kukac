
using Kukac.enums;
using Kukac.interfaces;
using System.Collections.Generic;
using System.Windows;

using System;
/// <summary>
/// Az osztály, amely az AI logikát implementálja a Kukac játékhoz.
/// </summary>
class CustomAI : Ai
{
    private Adat adat;
    private Test test;


    /// <summary>
    /// Az AI adatok inicializálása.
    /// </summary>
    /// <param name="adat">Az adatok objektuma</param>
    public void initAdat(Adat adat)
    {
        this.adat = adat;
    }

    /// <summary>
    /// Az AI inicializálása a Kukac objektummal.
    /// </summary>
    /// <param name="kukac">A Kukac objektum</param>
    public void initKukac(Test kukac)
    {
        test = kukac;
    }

    /// <summary>
    /// Az AI irányának beállítása.
    /// </summary>
    /// <returns>Az AI következő lépésének iránya</returns>

    public Iranyok setIrany()
    {
        Point foodPosition = adat.getEtel();
        Point headPosition = test.getFej();

        List<Point> path = AStarPathSearch(headPosition, foodPosition);

        if (path.Count > 1)
        {
            Point nextStep = path[1];

            int dx = (int)(nextStep.X - headPosition.X);
            int dy = (int)(nextStep.Y - headPosition.Y);

            return GetDirection(dx, dy);
        }

        return setDefault();
    }

    /// <summary>
    /// Az AI irányának beállítása.
    /// </summary>
    /// <returns>ha nincs elérhető útvonal az ételhez ez a funció hívodik meg</returns>
    public Iranyok setDefault()
    {
        Iranyok iranyok = test.getIrany();
        Point fej = test.getFej();

        // Determine the default direction based on the relative position of the food
        if (fej.X > adat.getEtel().X)
        {
            iranyok = Iranyok.BAL;
        }
        else if (fej.X < adat.getEtel().X)
        {
            iranyok = Iranyok.JOBB;
        }
        else if (fej.Y > adat.getEtel().Y)
        {
            iranyok = Iranyok.FEL;
        }
        else if (fej.Y < adat.getEtel().Y)
        {
            iranyok = Iranyok.LE;
        }

        Point point = new Point(fej.X + (double)iranyok.getXm(), fej.Y + (double)iranyok.getYm());
        int num = 1;

        while ((adat.getPalyaElem((int)point.X, (int)point.Y) == PalyaElemek.FAL || adat.getPalyaElem((int)point.X, (int)point.Y) == PalyaElemek.TEST) && num <= 4)
        {
            if (iranyok == Iranyok.FEL)
            {
                iranyok = Iranyok.JOBB;
            }
            else if (iranyok == Iranyok.JOBB)
            {
                iranyok = Iranyok.LE;
            }
            else if (iranyok == Iranyok.LE)
            {
                iranyok = Iranyok.BAL;
            }
            else if (iranyok == Iranyok.BAL)
            {
                iranyok = Iranyok.FEL;
            }

            point = new Point(fej.X + (double)iranyok.getXm(), fej.Y + (double)iranyok.getYm());
            num++;
        }

        return iranyok;
    }

    /// <summary>
    /// A* algoritmus használata a legközelebbi út megtalálásához egy adott célponthoz.
    /// </summary>
    /// <param name="start">A kezdőpont</param>
    /// <param name="goal">A cél</param>
    /// <returns>A legközelebbi út pontjainak listája</returns>
    private List<Point> AStarPathSearch(Point start, Point goal)
    {
        PriorityQueue<Point> openSet = new PriorityQueue<Point>();
        Dictionary<Point, Point> cameFrom = new Dictionary<Point, Point>();
        Dictionary<Point, int> gScore = new Dictionary<Point, int>();
        Dictionary<Point, int> fScore = new Dictionary<Point, int>();

        openSet.Enqueue(start, 0);
        gScore[start] = 0;
        fScore[start] = HeuristicCostEstimate(start, goal);

        while (openSet.Count > 0)
        {
            Point current = openSet.Dequeue();

            if (current == goal)
            {
                var path = ReconstructPath(cameFrom, current);
                return path;
            }

            List<Point> neighbors = GetValidNeighbors(current);

            foreach (Point neighbor in neighbors)
            {
                int tentativeGScore = gScore[current] + 1; 

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + HeuristicCostEstimate(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }

        return new List<Point>();
    }

    /// <summary>
    /// Két pont közötti heurisztikus költség becslése az euklideszi távolság felhasználásával.
    /// </summary>
    /// <param name="start">A kezdőpont.</param>
    /// <param name="goal">A célpont.</param>
    /// <returns>A becsült heurisztikus költség a kezdő- és célpont között.</returns>
    private int HeuristicCostEstimate(Point start, Point goal)
    {
        int dx = (int)(goal.X - start.X);
        int dy = (int)(goal.Y - start.Y);
        return (int)Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Az adott pont érvényes szomszédos pontjainak lekérése.
    /// </summary>
    /// <param name="point">A pont, amelynek szomszédjait le szeretnénk kérni.</param>
    /// <returns>Az érvényes szomszédos pontok listája.</returns>
    private List<Point> GetValidNeighbors(Point point)
    {
        List<Point> neighbors = new List<Point>();

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0, 0, 1, -1 };

        for (int i = 0; i < 4; i++)
        {
            int newX = (int)point.X + dx[i];
            int newY = (int)point.Y + dy[i];

            if (IsInBounds(newX, newY) && IsPositionValid(newX, newY))
            {
                neighbors.Add(new Point(newX, newY));
            }
        }

        return neighbors;
    }

    /// <summary>
    /// Ellenőrzi, hogy a megadott koordináták a játéktábla határain belül vannak-e.
    /// </summary>
    /// <param name="x">Az x-koordináta.</param>
    /// <param name="y">Az y-koordináta.</param>
    /// <returns>True, ha a koordináták a határok között vannak, egyébként false.</returns>
    private bool IsInBounds(int x, int y)
    {
        return x >= 0 && x < adat.getPalyaMeret().Width && y >= 0 && y < adat.getPalyaMeret().Height;
    }

    /// <summary>
    /// Ellenőrzi, hogy a megadott koordinátáknál található pozíció érvényes-e (azaz nem fal vagy kígyó által elfoglalt).
    /// </summary>
    /// <param name="x">Az x-koordináta.</param>
    /// <param name="y">Az y-koordináta.</param>
    /// <returns>True, ha a pozíció érvényes, egyébként false.</returns>
    private bool IsPositionValid(int x, int y)
    {
        PalyaElemek element = adat.getPalyaElem(x, y);
        return element != PalyaElemek.FAL && element != PalyaElemek.TEST;
    }

    /// <summary>
    /// Az útvonalat rekonstruálja a kezdőpozíciótól a megadott jelenlegi pozícióig.
    /// </summary>
    /// <param name="cameFrom">A pontokat és a hozzájuk tartozó előző pontokat tároló szótár.</param>
    /// <param name="current">A jelenlegi pozíció.</param>
    /// <returns>Az útvonalat reprezentáló pontok listája.</returns>
    private List<Point> ReconstructPath(Dictionary<Point, Point> cameFrom, Point current)
    {
        var path = new List<Point>();
        path.Add(current);

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }

        return path;
    }

    /// <summary>
    /// Az adott x- és y-koordináta változást átalakítja iránnyá.
    /// </summary>
    /// <param name="dx">Az x-koordináta változás.</param>
    /// <param name="dy">Az y-koordináta változás.</param>
    /// <returns>Az adott koordináta változásnak megfelelő irány.</returns>
    private Iranyok GetDirection(int dx, int dy)
    {
        if (dx == 1)
            return Iranyok.JOBB;
        else if (dx == -1)
            return Iranyok.BAL;
        else if (dy == 1)
            return Iranyok.LE;
        else if (dy == -1)
            return Iranyok.FEL;

        return setDefault();
    }
}

/// <summary>
/// Bináris kupacot használó prioritási sor megvalósítása.
/// </summary>
/// <typeparam name="T">A prioritási sorban tárolt elemek típusa.</typeparam>
public class PriorityQueue<T>
{
    private List<Tuple<T, int>> heap;
    private Dictionary<T, int> itemIndices;

    public int Count { get { return heap.Count; } }

    public PriorityQueue()
    {
        heap = new List<Tuple<T, int>>();
        itemIndices = new Dictionary<T, int>();
    }


    /// <summary>
    /// Elemet besorol a megadott prioritással.
    /// </summary>
    /// <param name="item">A besorolandó elem</param>
    /// <param name="priority">Az elem prioritása</param>


    public void Enqueue(T item, int priority)
    {
        heap.Add(new Tuple<T, int>(item, priority));
        itemIndices[item] = heap.Count - 1;

        int currentIndex = heap.Count - 1;

        while (currentIndex > 0)
        {
            int parentIndex = (currentIndex - 1) / 2;

            if (heap[parentIndex].Item2 <= heap[currentIndex].Item2)
                break;

            Swap(currentIndex, parentIndex);
            currentIndex = parentIndex;
        }
    }



    /// <summary>
    /// Kiveszi a legmagasabb prioritású elemet a sorból.
    /// </summary>
    /// <returns>A legmagasabb prioritású elem.</returns>
    public T Dequeue()
    {
        if (heap.Count == 0)
            throw new InvalidOperationException("Priority queue is empty.");

        T item = heap[0].Item1;
        int lastIndex = heap.Count - 1;

        heap[0] = heap[lastIndex];
        itemIndices[heap[0].Item1] = 0;
        heap.RemoveAt(lastIndex);
        itemIndices.Remove(item);

        int currentIndex = 0;

        while (true)
        {
            int leftChildIndex = 2 * currentIndex + 1;
            int rightChildIndex = 2 * currentIndex + 2;

            if (leftChildIndex >= heap.Count)
                break;

            int minIndex = leftChildIndex;

            if (rightChildIndex < heap.Count && heap[rightChildIndex].Item2 < heap[leftChildIndex].Item2)
                minIndex = rightChildIndex;

            if (heap[currentIndex].Item2 <= heap[minIndex].Item2)
                break;

            Swap(currentIndex, minIndex);
            currentIndex = minIndex;
        }

        return item;
    }

     ///<summary>
     ///   Ellenőrzi, hogy a prioritási sor tartalmazza-e a megadott elemet.
    ///</summary>
    ///<param name="item">Az ellenőrizendő elem.</param>
    ///<returns>True, ha az elem megtalálható a prioritási sorban, különben false.</returns>

    public bool Contains(T item)
    {
        return itemIndices.ContainsKey(item);
    }

    /// <summary>
    /// Kicseréli a megadott indexeken található elemeket a halomban.
    /// </summary>
    /// <param name="index1">Az első elem indexe.</param>
    /// <param name="index2">A második elem indexe.</param>

    private void Swap(int index1, int index2)
    {
        Tuple<T, int> temp = heap[index1];
        heap[index1] = heap[index2];
        heap[index2] = temp;

        itemIndices[heap[index1].Item1] = index1;
        itemIndices[heap[index2].Item1] = index2;
    }
}
