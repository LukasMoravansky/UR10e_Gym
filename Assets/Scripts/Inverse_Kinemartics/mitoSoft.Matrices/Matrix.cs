using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace mitoSoft.Matrices
{
    [DebuggerDisplay("Rows:{RowCount};Colums{ColumnCount}")]
    public class Matrix
    {
        private readonly decimal[,] _matrix;

        public Matrix(int rows, int columns)
        {
            this._matrix = new decimal[rows, columns];
        }

        public decimal this[int row, int column]
        {
            get => this._matrix[row, column];
            set => this._matrix[row, column] = value;
        }

        public int ColumnCount => this._matrix.GetUpperBound(1) + 1;

        public int RowCount => this._matrix.GetUpperBound(0) + 1;

        public static bool TryParse(IList<string> lines, out Matrix matrix)
        {
            try
            {
                matrix = Parse(lines);
                return true;
            }
            catch
            {
                matrix = null;
                return false;
            }
        }

        public static Matrix Parse(IList<string> lines)
        {
            var columnCount = GetColumnCount(lines);

            var matix = new Matrix(lines.Count, columnCount);

            for (int row = 0; row < lines.Count; row++)
            {
                var rowValues = lines[row].Split(';');

                for (int column = 0; column < columnCount; column++)
                {
                    var value = decimal.Parse(rowValues[column]);

                    matix[row, column] = value;
                }
            }

            return matix;
        }

        public static int GetColumnCount(IEnumerable<string> lines)
        {
            int columnCount = -1;
            foreach (var line in lines)
            {
                if (columnCount >= 0 && line.Split(';').Count() != columnCount)
                {
                    throw new InvalidOperationException($"Invalid Column count in line {line}.");
                }
                columnCount = line.Split(';').Count();
            }
            return columnCount;
        }

        public  decimal GetColumnSum( int columnIndex)
        {
            decimal sum = 0;
            for (int row = 0; row < this.RowCount; row++)
            {
                sum += this[row, columnIndex];
            }
            return sum;
        }

        public  decimal GetRowSum( int rowIndex)
        {
            decimal sum = 0;
            for (int col = 0; col < this.ColumnCount; col++)
            {
                sum += this[rowIndex, col];
            }
            return sum;
        }
    }
}
